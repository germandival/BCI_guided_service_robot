# Jonas Braun
# jonas.braun@tum.de
# 19.01.2019

from time import sleep
import sys

sys.path.extend(['C:\\PYTHON_DATA\\20_NISE', 'C:/PYTHON_DATA/20_NISE'])

from communication.udp import *
from execution.commands import BCICommands


class InterfaceTest:
    def __init__(self):
        """
        old version of clas BCIStatemachine() in order to test UDP communication
        """
        self.comClient = UDPClient(server_ip="192.168.137.119", server_port=5005)  # do not change
        self.comServer = UDPServer(server_name=server_thread, ip="192.168.137.1", port=5005)

        self.CMD = BCICommands()

        self.sleep_main = 1
        self.sleep_confirm = 2

    def start(self):
        print('start BCI state machine')
        self.comServer.start()
        self.loop()

    def loop(self):
        i = 1
        # command reception loop
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
                print('class result in loop is {}'.format(result))
                i += 1
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
                            break
            # wait 1 second until pulling whether new command has arrived again
            sleep(self.sleep_main)

    @staticmethod
    def start_bci(screen_id, i=0):
        print('I will demand screen ID {} from OpenVibe.'.format(screen_id))
        list = [1, 4, 1, 1, 4, 1, 1, 4, 1, 1, 4, 1, 1, 4, 1, 1, 4, 1, 1, 4, 1, 1, 4, 1]
        # [6,4,1,4,1,2,4,3,1,1,2,1,3,4,3,1,4,5,1,4,2]  # test sequence

        sleep(10)
        class_result = np.random.randint(low=1, high=5)
        if i:
            class_result = list[i - 1]
        return class_result

    def stop(self):
        pass


if __name__ == '__main__':

    myBCI = InterfaceTest()
    myBCI.start()
