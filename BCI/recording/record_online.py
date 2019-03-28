# Jonas Braun
# jonas.braun@tum.de
# 18.01.2019

# record P300 data from LSL created by OpenVibe

import os
import sys

import threading  # NOQA
import scipy.io as sio  # NOQA
import pylsl  # NOQA
import time  # NOQA

import math
import numpy as np


# class defining stimulation values used by openvibe
class StimValues:
    def __init__(self):
        self.exp_over = 32770
        self.exp_start = 32769
        self.target = 33285
        self.notarget = 33286
        self.rows = [33025, 33026, 33027]
        self.columns = [33031, 33032, 33033]
        self.trial_start = 32773
        self.trial_end = 32774

        self.clock = 1
        self.clock_time = 0.004


class NoRecordingDataError(Exception):
    def __init__(self):
        self.value = "Received no data while recording"

    def __str__(self):
        return repr(self.value)


def time_str():
    return time.strftime("%H_%M_%d_%m_%Y", time.gmtime())


def record(stop_event, channel_data=[], time_stamps=[], Fs=[], stream_name='openvibeSignal'):
    """
    function to be run in a thread to record data from lsl
    :param stop_event: threading.event that can be set to stop the server
    :param channel_data: buffer that samples of channel data will be appended to
    :param time_stamps: corresponding time stamps
    :param Fs: sampling frequency
    :param stream_name: name of the stream to be searched for
    :return:
    """
    print('The recording thread started.')
    # 1. get input stream
    streams = pylsl.resolve_stream('name', stream_name)
    print('Found stream ', stream_name, ': ', streams)
    inlet = pylsl.stream_inlet(streams[0])
    inletInfo = inlet.info()
    inlet_sampleRate = int(inletInfo.nominal_srate())
    inlet_numChannels = int(inletInfo.channel_count())
    Fs = inlet_sampleRate
    print("Reported sample rate: %i , number of channels: %i" % (inlet_sampleRate, inlet_numChannels))

    while not stop_event.is_set():
        try:
            sample, time_stamp = inlet.pull_sample()
            time_stamp += inlet.time_correction()

            time_stamps.append(time_stamp)
            channel_data.append(sample)
            if len(time_stamps) % 1000 == 0:
                print("Length of recorded data is ", len(time_stamps))

        except KeyboardInterrupt:
            # save data and exit on KeybordInterrupt
            complete_samples = min(len(time_stamps), len(channel_data))
            sio.savemat("recording_" + time_str() + ".mat", {
                "time_stamps": time_stamps[:complete_samples],
                "channel_data": channel_data[:complete_samples],
            })
            break


def get_markers(stop_event, channel_time=[], markers=[], markers_time=[], start_proc=[], stream_name='openvibeMarkers'):
    """
    function to be run in a thread to record markers from lsl
    :param stop_event: threading.event that can be set to stop the server
    :param channel_time: corresponding time stamps to data sample
    :param markers: buffer to append the markers to
    :param markers_time: buffer to append correponding time stamps to
    :param start_proc: buffer that True will be appended to once the experiment is over and processing can be started
    :param stream_name: name of the stream to be searched for
    :return:
    """
    codes = StimValues()
    print('The event thread started.')
    # 1. get input stream
    streams = pylsl.resolve_stream('name', stream_name)
    print('Found stream ', stream_name, ': ', streams)
    inlet = pylsl.stream_inlet(streams[0])
    inletInfo = inlet.info()
    inlet_sampleRate = int(inletInfo.nominal_srate())
    inlet_numChannels = int(inletInfo.channel_count())

    print("Reported sample rate: %i , number of channels: %i" % (inlet_sampleRate, inlet_numChannels))

    curr_time = 0.0
    exp_start = False
    while not stop_event.is_set():
        try:
            sample, time_stamp = inlet.pull_sample()

            if exp_start and sample[0] == codes.clock:
                # if a clock marker has been received update current clock.
                # this is required, because marker stream comes without time stamps
                curr_time += codes.clock_time
            elif not exp_start and sample[0] == codes.exp_start:
                # if experiment start, synchronise to data stream and append
                print('Received Marker EXPERIMENT START')
                exp_start = True
                curr_time = channel_time[-1]
                markers_time.append(curr_time)
                markers.append(sample[0])
            elif exp_start and sample[0] == codes.exp_start:
                print('Received Marker EXPERIMENT START')
                markers_time.append(curr_time)
                markers.append(sample[0])
            elif exp_start and sample[0] == codes.exp_over:
                # if experiment end. append sample and append true, so that it can be detected in main loop
                print('Received Marker EXPERIMENT STOP')
                markers_time.append(curr_time)
                markers.append(sample[0])
                start_proc.append(True)
            elif exp_start:
                # if normal sample append with synchronised time
                markers_time.append(curr_time)
                markers.append(sample[0])
                if len(markers_time) % 1000 == 0:
                    print("Number of recorded events is ", len(markers_time))

        except KeyboardInterrupt:
            # save data and exit on KeybordInterrupt
            complete_samples = min(len(markers_time), len(markers))
            sio.savemat("markers_" + time_str() + ".mat", {
                "markers_time": markers_time[:complete_samples],
                "markers": markers[:complete_samples],
            })
            break


# class reading data and markers from LSL
class RecordData:
    def __init__(self, age=23, gender="male",
                 record_func=record, marker_func=get_markers, nodataerroron=False):

        # list including all the recorded EEG data
        self.X = []
        # all time stamps, one is added for each data point
        self.time_stamps = []
        self.time_markers = []
        # label of each trial: 0: left, 1: right, 2: both
        self.markers = []
        # sampling frequency
        self.Fs = []

        # list to indicate that experiment is over
        # used in property exp_over
        self.start_proc = [False]

        self.gender = gender
        self.age = age

        # define whether NoRecordingDataError will be triggered
        self.nodataerroron = nodataerroron
        self.record_func = record_func
        self.marker_func = marker_func

        # initialise a subfolder where the data is to be saved
        # if it does not yet exist, create it
        self.datapath = os.path.join(os.getcwd(), '00_DATA')
        if not os.path.exists(self.datapath):
            os.makedirs(self.datapath)

            # stop event used to pause and resume to the recording & processing thread
        self.stop_event_rec = threading.Event()
        self.stop_event_mark = threading.Event()

        # initialise recording thread. It does not run yet. Therefore use start_recording()
        recording_thread = threading.Thread(
            target=self.record_func,
            args=(self.stop_event_rec, self.X, self.time_stamps, self.Fs),
        )
        recording_thread.daemon = True
        self.recording_thread = recording_thread

        # initialise markers thread. It does not run yet. Therefore use start_recording()
        markers_thread = threading.Thread(
            target=self.marker_func,
            args=(self.stop_event_mark, self.time_stamps, self.markers, self.time_markers,
                  self.start_proc),
        )
        markers_thread.daemon = True
        self.markers_thread = markers_thread

    def __iter__(self):
        yield 'age', self.age
        yield 'X', self.X
        yield 'time_stamps', self.time_stamps
        yield 'time_markers', self.time_markers
        yield 'markers', self.markers
        yield 'Fs', self.Fs
        yield 'gender', self.gender
        yield 'start_proc', self.start_proc

    @property
    def exp_over(self):
        return self.start_proc[-1]

    def start_recording(self, len_X=0, len_Y=0):
        # start the recording thread
        self.recording_thread.start()
        """
        ns = 10
        time.sleep(ns)
        # check whether data arrived, if not raise error
        if len(self.X) - len_X == 0:
            print('Did not receive any eeg data from LSL within {:d} seconds.\n'.format(ns))
            if self.nodataerroron:
                raise NoRecordingDataError()
        """
        self.markers_thread.start()
        """
        time.sleep(ns)
        # check whether data arrived, if not raise error
        if len(self.markers) - len_Y == 0:
            print('Did not receive any marker data from LSL within {:d} seconds.\n'.format(ns))
            if self.nodataerroron:
                raise NoRecordingDataError()
        """

    def pause_recording(self):
        # raise stop_event to break the loop in record()
        self.stop_event_rec.set()
        self.stop_event_mark.set()
        print('Recording has been paused.')

    def restart_recording(self):
        # newly initialise the recording thread and start it
        self.stop_event_rec.clear()
        self.stop_event_mark.clear()

        recording_thread = threading.Thread(
            target=self.record_func,
            args=(self.stop_event_rec, self.X, self.time_stamps, self.Fs),
        )
        recording_thread.daemon = True
        self.recording_thread = recording_thread

        # initialise markers thread. It does not run yet. Therefore use start_recording()
        markers_thread = threading.Thread(
            target=self.marker_func,
            args=(self.stop_event_mark, self.time_stamps, self.markers, self.time_markers),
        )
        markers_thread.daemon = True
        self.markers_thread = markers_thread
        print('Recording has been restarted.')

    def stop_recording_and_save(self, file_name="EEG_session_" + time_str() + ".mat"):
        # finish the recording, save all data to a .mat file
        self.pause_recording()
        sio.savemat(os.path.join(self.datapath, file_name), dict(self))
        print('Recording will stop.')
        return file_name, self.datapath

    def save_data(self, file_name="EEG_session_" + time_str() + ".mat"):
        # only save data while still keeping the recording thread alive
        sio.savemat(os.path.join(self.datapath, file_name), dict(self))
        return file_name, self.datapath

    def get_last_trial(self):
        """
        function inmportant for online processing. returns eeg data, corresponding time stamps
        markers + corrsponding time stamps of the last trial
        :return: 
        """
        self.start_proc.append(False)
        start_m = len(self.markers) - 1 - self.markers[::-1].index(StimValues().exp_start)
        stop_m = len(self.markers) - 1 - self.markers[::-1].index(StimValues().exp_over)
        # start_d = self.time_stamps.index(self.time_markers[start_m])
        start_d = np.argmin(np.abs(np.array(self.time_stamps) - self.time_markers[start_m]))
        # stop_d = self.time_stamps.index(self.time_markers[stop_m])
        stop_d = np.argmin(np.abs(np.array(self.time_stamps) - self.time_markers[stop_m]))
        data = self.X[start_d:stop_d]
        times = self.time_stamps[start_d:stop_d]
        markers = self.markers[start_m:stop_m]
        marker_times = self.time_markers[start_m:stop_m]
        return data, times, markers, marker_times

