# Jonas Braun
# jonas.braun@tum.de
# 18.01.2019
# example to create a UDP server


import socket
UDP_IP = "192.168.137.1"  # socket.gethostbyname(socket.gethostname())
print(UDP_IP)
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

sock.bind((UDP_IP, UDP_PORT))
print('bound socket')
data = []
addr = []
while True:
    print('start listening')
    data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
    print("received message:", data)

