# contains the client that connects to the Jetson Nano's server

import socket
import struct
import sys

# initializes client socket on instantiation, 
# must then connect to a server with connect method
class Client:
    def __init__(self):
        self.client = self.init_client()

    def init_client(self):
        # initializes a network socket
        try:
            print("Creating socket...")
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("success.")
        except socket.error as err:
            print("error:", err)
            sys.exit(1)

        return client
    
    def connect(self, ip, port):
        try:
            # connects to the host at host_ip:host_port
            print(f"Connecting to {ip}:{port}...")
            self.client.connect((ip, port))
            print("connected.")
        except socket.error as err:
            print("error:", err)
            sys.exit(1)

    # sends a message for robot movement.
    # 
    # speed, turn: percents in range [-100, 100]
    # duration: milliseconds in range [0, 4,294,967,295]
    # controlA: int in range [0, 255], set 1 to terminate connection
    # controlB: int in range [0, 255], unused (network messages are best in powers of 2)
    def send_movement(self, speed=0, turn=0, duration=20, controlA=0, controlB=0):
        # packs the five parameters into an 8-byte binary string
        # ">bbIBB" = big-endian, signed byte, signed byte, unsigned 4-byte int, unsigned byte, unsigned byte
        message = struct.pack(">bbIbb", speed, turn, duration, controlA, controlB)
        
        self.client.send(message)
            
    def close(self):
        self.client.close()
        print("Connection closed.")




