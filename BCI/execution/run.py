# Jonas Braun
# jonas.braun@tum.de
# 12.01.2019

# main to run a BCI experiment with first offline recording and then online recording
import sys
sys.path.extend(['C:\\PYTHON_DATA\\20_NISE', 'C:/PYTHON_DATA/20_NISE'])

from recording.record_offline_P300 import RecordData
from processing.process_offline import Dataset, Params
from execution.states import BCIStateMachine
from time import sleep
import os

if __name__ == '__main__':
    print('Started BCI Program. Please set up an OpenViBE acquisition server for the EEG data.')
    RecOffline = RecordData()
    RecOffline.start_recording()
    print('Started recording. Please start a paradigm in OpenViBE to record training data')
    while not RecOffline.exp_over:
        sleep(5)

    filename, path = RecOffline.stop_recording_and_save()
    print('Stopped recording. File is at path {} with name {}'.format(path, filename))
    print('Will now train classifier.')
    myParams = Params()
    myDS = Dataset(filename=[os.path.join(path, filename)], Params=myParams)
    myDS.process()
    myDS.plot_channel()
    clf_ok = input('Are these cv scores and channel data ok? If so, enter 1 , if not enter 0')

    while not clf_ok:
        RecOffline.restart_recording()
        print('Restarted recording. Please start a paradigm in OpenViBE to record training data')
        while not RecOffline.exp_over:
            sleep(5)

        filename, path = RecOffline.stop_recording_and_save()
        print('Stopped recording. Will now train classifier.')
        myDS = Dataset(filename=os.path.join(path, filename), Params=myParams)
        myDS.process()
        myDS.plot_channel()
        clf_ok = input('Are these cv scores and channel data ok? If so, enter 1 , if not enter 0')

    # start online experiment
    print('Will now start online experiment')
    myBCI = BCIStateMachine(test_seq=[1, 4], EEG=True, Dataset=myDS)
    myBCI.start()
