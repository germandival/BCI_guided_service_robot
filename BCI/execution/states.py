# Jonas Braun
# jonas.braun@tum.de
# 18.01.2019

from time import sleep
import sys
from subprocess import check_output
sys.path.extend(['C:\\PYTHON_DATA\\20_NISE', 'C:/PYTHON_DATA/20_NISE'])

from communication.udp import *
from execution.commands import BCICommands
from recording.lsl_send import StreamData
from recording.record_online import RecordData
from processing.process_online import Model
from processing.process_offline import Dataset


class BCIStateMachine:
    def __init__(self, test_seq=None, EEG=True, Dataset=Dataset()):
        """

        :param test_seq: list containg integers that are sent as result back to the robot insteado of the actual reault.
                         can be used for testing
        :param EEG: boolean. If true, OpenViBE will be activated while testing, if not, result specified in test_seq
                    will be returned immedeately
        :param Dataset: instance of class Dataset() that has offline data loaded and processed. will be uses as
                        classifier for online trials
        """
        # initialise all servers and recording threads
        self.comClient = UDPClient(server_ip="192.168.137.119", server_port=5005)
        self.comServer = UDPServer(server_name=server_thread, ip="192.168.137.1", port=5005)
        self.vidServer = VideoServer(video_size=(100, 100), ip="192.168.137.1", port=5006)
        self.LSLout = StreamData()
        self.LSLin = RecordData()
        # initialise online classifier
        self.model = Model(Dataset=Dataset)

        self.CMD = BCICommands()

        # define time constants
        self.sleep_main = 1  # pause time in main loop
        self.sleep_confirm = 2   # pause time to wait for confirmation of result
        self.certainty_accept = 0.9  # certainty required to accept a BCI classification
        self.max_iter_cert = 5  # maximum number of repetitions to increase certainty

        self.test_seq = test_seq
        self.EEG = EEG

    def start(self):
        """
        start the BCI session
        :return:
        """
        print('start BCI state machine')
        # start all separate threads
        self.comServer.start()
        self.LSLout.start()
        self.LSLin.start_recording()
        self.vidServer.start()
        check_output("cd C:\\PYTHON_DATA\\20_NISE", shell=True)
        # give instructions
        print('Add an OpenViBE acquisition client to record the data stream BCI_data and the marker stream BCI_select')
        print('Start the OpenViBE Online Scenario with the number of repetitions that are desired.')
        print('IMPORTANT: set the automatic restart option.')
        start = input('After it finished once, confirm to start by entering 1. If you want to stop, enter 0')
        if start:
            self.loop()
        else:
            self.stop()

    def loop(self):
        """
        run the loop waiting for commands from the robot
        :return:
        """
        # command reception loop
        i = 0
        print('Ready to loop. Waiting for commands.')
        while True:
            # wait for a command to come in
            if self.comServer.isnewdata():
                # confirm reception of command
                self.comClient.send(self.CMD.confirm)
                # convert command to integer indicating screen number
                screen = self.CMD.screen(self.comServer.get_lastdata())
                print('recieved command to start screen ', screen)
                # start the BCI based on the command
                result = self.start_bci(screen, i)
                i = i + 1
                # confirmation loop
                while True:
                    # send result of BCI
                    print('sent command with result ', result)
                    self.comClient.send(self.CMD.result(result))
                    # wait for some time to allow confirmations of result
                    sleep(self.sleep_confirm)
                    # check whether result has been confirmed
                    # if so break and start again waiting for next command
                    if self.comServer.isnewdata():
                        data = self.comServer.get_lastdata()
                        print(data)
                        if data == self.CMD.confirm:
                            sleep(5)
                            break
            # wait 1 second until pulling whether new command has arrived again
            sleep(self.sleep_main)
            if self.test_seq is not None:
                if i == len(self.test_seq):
                    print('Test is over. Will shut down in 5 minute.')
                    sleep(5*60)
                    break

        self.stop()

    def start_bci(self, screen_id, i=0):
        """
        perform a BCI online trial to get answer to the question posed by the robot
        :param screen_id: the number of the screen that the robot demanded
        :param i: number of loop. uesd for testing if test_seq is specified
        :return: class_result: result of BCI classification reflecting patient intent
        """
        print('I will demand screen ID {} from OpenVibe.'.format(screen_id))
        # copy correct P300 interface to the location where openvibe loads from
        cmd_string = "copy p300-{:1d}.ui p300-0.ui".format(screen_id)
        check_output(cmd_string, shell=True)

        certainty = 0.0
        class_result = 0
        n_rep = 0
        while certainty < self.certainty_accept:
            # make sure no previous data blocks the flag
            self.LSLin.start_proc.append(False)
            # tell openvibe to stop scenario. thanks to automatic restart, it will restart with new interface
            self.LSLout.add_stim(0)


            # do BCI experiment or just return test sequence result
            if self.test_seq is not None and not self.EEG:
                sleep(5)
                class_result = self.test_seq[i]
                certainty = 1
                # class_result = np.random.randint(low=1, high=5)
            else:
                # wait for BCI to return result
                while not self.LSLin.exp_over:
                    sleep(1)
                # get data from last trial
                data, time, markers, marker_time = self.LSLin.get_last_trial()
                print('received data of length {} and markers of length {}. Will now classify.'.format(len(time),
                                                                                                       len(marker_time)))
                # start online rocessing
                class_result, certainty = self.model.classify_online(data, time, markers, marker_time, n_rep)
                print('classification result is {} with certainty {}'.format(class_result, certainty))
                if self.test_seq is not None:
                    # overwrite result if test sequence is specified
                    class_result = self.test_seq[i]
                    certainty = np.random.uniform(low=0.88, high=1.0, size=1)
                if certainty < self.certainty_accept:
                    if n_rep+1 == self.max_iter_cert:
                        print('max certainty out of 5 repetitions is {}.'.format(certainty))
                        print('Will return 0.')
                        class_result = 0
                        break
                    n_rep += 1
                    print('Certainty not high enough, will collect more data.')
        return class_result

    def stop(self):
        """
        stop all threads
        :return:
        """
        self.LSLin.stop_recording_and_save()
        self.LSLout.stop()
        self.comServer.stop()
        self.vidServer.stop()

