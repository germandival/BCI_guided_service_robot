# Jonas Braun
# jonas.braun@tum.de
# 13.01.2019

# sending and receiving data via UDP
import socket
import threading

import matplotlib.pyplot as plt
import numpy as np
from time import sleep


# UDP client to send data
class UDPClient:
    def __init__(self, server_ip="192.168.137.119", server_port=5005):
        self.server_ip = server_ip
        self.server_port = server_port
        print("UDP target IP:", self.server_ip)
        print("UDP target port:", self.server_port)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, message, ntimes=1):
        """
        convert message to bytes and send via UDP
        :param message: message to be sent
        :param ntimes: how many times should message be repeated
        :return:
        """
        if isinstance(message, str):
            message = bytes(message, 'UTF-8')
        elif isinstance(message, int):
            message = bytes(str(message), 'UTF-8')
        elif isinstance(message, float):
            message = bytes(str(message), 'UTF-8')
        elif isinstance(message, np.ndarray):
            message = message.tobytes()
        for i in range(ntimes):
            self.sock.sendto(message, (self.server_ip, self.server_port))
        print('sent message: ', message)


def server_thread(stop_event, ip, port, data):
    """
    function to be run in a separate thread to create a UDP server
    :param stop_event:  threading.event that can be set to stop the server
    :param ip: server ip
    :param port: server port
    :param data: buffer of data that server will append the data received to. Should be read as __iter__
    :return:
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print('Started UDP socket under IP address', ip)
    while not stop_event.is_set():
        cmd, addr = sock.recvfrom(65507)  # buffer size 1024
        data.append(cmd)
        print('received: ' + str(cmd))

    sock.close()
    print('Shut down server with IP address ', ip, ' and port ', port)


# class implementing a UDP server to receive messages via UDP
class UDPServer:
    def __init__(self, server_name=server_thread, ip="192.168.137.1", port=5005):
        self.ip = ip  # socket.gethostbyname(socket.gethostname())
        self.port = port
        self.server_name = server_name
        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock.bind((self.ip, self.port))
        self.data = []
        self.n_dataread = 0

        self.stop_event = threading.Event()
        # initialise recording thread. It does not run yet. Therefore use start_recording()
        server = threading.Thread(
            target=self.server_name,
            args=(self.stop_event, self.ip, self.port, self.data),
        )
        server.daemon = True
        self.server = server

    def __iter__(self):
        yield 'data', self.data

    @property
    def buffer_len(self):
        return len(self.data)

    def start(self):
        self.server.start()

    def stop(self):
        self.stop_event.set()

    def isnewdata(self):
        if self.buffer_len > self.n_dataread:
            return True
        else:
            return False

    def get_lastdata(self):
        self.n_dataread = self.buffer_len
        return self.data[-1]


def video_read_thread(stop_event, ip, port, buffer=[], video_size=(100, 100)):
    """
    function to be run in a separate thread to create a UDP server that receives a video stream
    :param stop_event:  threading.event that can be set to stop the server
    :param ip: server ip
    :param port: server port
    :param buffer: buffer of frames that server will append the data received to. Should be read as __iter__
    :param: video_size: predefine the size of the video to be received. required to reshape bit stream
    :return:
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print('Started UDP socket under IP address ', ip, ' and port ', port)
    # fig, ax = plt.subplots(1, 1)
    # plt.ion()
    while not stop_event.is_set():
        msg, addr = sock.recvfrom(65507)  # buffer size 1024
        tmp = np.frombuffer(msg, dtype=np.uint8)
        # print('received np array of size ', tmp.shape)
        img = np.reshape(tmp, video_size)
        buffer.append(img)
        # limit buffer size
        if len(buffer) > 2:
            del buffer[0:-2]

    sock.close()
    print('Shut down server with IP address')


def video_show_thread(stop_event, buffer):
    """
    separate thread to show a video by reading from buffer using matplotlib
    :param stop_event: threading.event that can be set to stop the server
    :param buffer: buffer of data that frames shoudl be read from. will always read last frame
    :return:
    """
    plt.ion()
    while not stop_event.is_set():
            plt.imshow(buffer[-1], cmap='gray')
            plt.pause(0.0000001)
    plt.ioff()

# class implementing a video server to read video from UDP stream and show it using matplotlib
class VideoServer:
    def __init__(self, server_name=video_read_thread, show_name=video_show_thread, ip="192.168.137.1", port=5006,
                 video_size=(100, 100)):
        self.ip = ip
        self.port = port
        self.server_name = server_name
        self.video_size = video_size
        self.buffer = [np.random.randint(0, 255, video_size)]
        self.stop_event = threading.Event()
        # initialise recording thread. It does not run yet. Therefore use start_recording()
        server = threading.Thread(
            target=self.server_name,
            args=(self.stop_event, self.ip, self.port, self.buffer, self.video_size),
        )
        server.daemon = True
        self.server = server

        self.stop_event_show = threading.Event()

        server = threading.Thread(
            target=show_name,
            args=(self.stop_event_show, self.buffer),
        )
        server.daemon = True
        self.server_show = server

    def __iter__(self):
        yield 'buffer', self.buffer

    def start(self):
        self.server.start()
        self.server_show.start()

    def stop(self):
        self.stop_event.set()
        self.stop_event_show.set()


def send_random_video(ip='192.168.137.1', port=5006, size=(100, 100), f=5, t_max=30):
    """
    function to test the UDP video server by sending a video with random pixels
    :param ip: video server ip
    :param port: video server port
    :param size: video frame size
    :param f: video frame rate
    :param t_max: end time of video sending
    :return:
    """
    myCli = UDPClient(server_ip=ip, server_port=port)
    for i in range(f*t_max):
        myCli.send(np.uint8(np.random.randint(low=0, high=255, size=size)))
        sleep(1/f)


if __name__ == '__main__':
    myVid = VideoServer()
    myVid.start()
