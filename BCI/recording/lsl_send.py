# Jonas Braun
# jonas.braun@tum.de
# 18.01.2019

# send markers and data via LSL

import os
import sys

import threading  # NOQA
import scipy.io as sio  # NOQA
import pylsl
import numpy as np
import time  # NOQA


def send_markers(stop_event, markers=[], stream_name='BCI_select'):
    """
    function to be run in a thread to send markers via lsl
    :param stop_event: threading.event that can be set to stop the server
    :param markers: buffer of markers. will send marker if new one is appended to markers
    :param stream_name: name of stream to be generated
    :return:
    """
    print('The lsl BCI selection thread started.')
    # 1. create a new stream
    info = pylsl.StreamInfo(name=stream_name, type='marker',
                            channel_count=1, channel_format='int32')
    # 2. make an outlet
    outlet = pylsl.StreamOutlet(info)
    print('Created marker stream :', outlet)
    stim_sent = 0
    while not stop_event.is_set():
        try:
            if len(markers) > stim_sent:
                stimulus = markers[-1]
                print('will send stimulus ', stimulus)
                outlet.push_sample([np.int32(stimulus)])
                stim_sent += 1
                time.sleep(5)

        except KeyboardInterrupt:
            break
    print('lsl send shutdown complete')


def send_data(stop_event, markers=[], stream_name='BCI_data'):
    """
        function to generated random data to send via LSL. makes it easier to send markers in parralel
        :param stop_event: threading.event that can be set to stop the server
        :param markers: buffer of markers. will send marker if new one is appended to markers
        :param stream_name: name of stream to be generated
        :return:
        """
    print('The lsl BCI selection thread started.')
    # 1. create a new stream
    info = pylsl.StreamInfo(name=stream_name, type='data',
                            channel_count=1, nominal_srate=100,
                            channel_format='float32')
    # 2. make an outlet
    outlet = pylsl.StreamOutlet(info)
    print('Created data stream :', outlet)

    while not stop_event.is_set():
        try:
            mysample = [np.float32(np.random.rand(1, 1))]
            outlet.push_sample(mysample)
            time.sleep(0.01)

        except KeyboardInterrupt:
            break
    print('lsl send shutdown complete')


# class to generate both marker and data stream on LSL
class StreamData:
    def __init__(self, send_func=send_markers, data_func=send_data):

        # list including all markers that have been sent
        self.markers = []
        self.send_func = send_func
        self.data_func = data_func

        # stop event used to pause and resume to the recording & processing thread
        self.stop_event = threading.Event()
        self.stop_event_data = threading.Event()
        # initialise streaming thread. It does not run yet. Therefore use start()
        data_thread = threading.Thread(
            target=self.data_func,
            args=(self.stop_event_data, self.markers),
        )
        data_thread.daemon = True
        self.data_thread = data_thread

        send_thread = threading.Thread(
            target=self.send_func,
            args=(self.stop_event, self.markers),
        )
        send_thread.daemon = True
        self.send_thread = send_thread

    def __iter__(self):
        yield 'markers', self.markers

    def start(self):
        # start the thread
        self.send_thread.start()
        self.data_thread.start()
        print('LSL Sending has been started.')

    def stop(self):
        # raise stop_event to break the loop in record()
        self.stop_event.set()
        print('LSL Sending has been paused.')

    def restart(self):
        # newly initialise the recording thread and start it
        self.stop_event.clear()

        send_thread = threading.Thread(
            target=self.send_func,
            args=(self.stop_event, self.markers),
        )
        send_thread.daemon = True
        self.send_thread = send_thread
        print('LSL Sending has been restarted.')

    def add_stim(self, code):
        """
        evoke this function if you want to send a marker via LSL
        :param code: integer that is to be sent via LSL
        :return:
        """
        self.markers.append(code)

