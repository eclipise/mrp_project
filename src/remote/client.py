# contains the client that connects to the Jetson Nano's server

import socket
import struct
import sys

# initializes client socket on instantiation, 
# must then connect to a server with connect method
class Client:
    def __init__(self):
        self.client = self.init_client()

    # initializes a network socket
    def init_client(self):
        try:
            print("Creating socket...")
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("success.")
        except socket.error as err:
            print("error:", err)
            sys.exit(1)

        return client
    
    # connects to the host at ip:port
    def connect(self, ip, port):
        try:
            print(f"Connecting to {ip}:{port}...")
            self.client.connect((ip, port))
            print("connected.")
        except socket.error as err:
            print("error:", err)
            sys.exit(1)

    # Sends a single byte for overall program control (i.e. exiting)
    # 
    # message: int [0, 255]
    #       0: continue
    #       1: terminate
    def send_control(self, message):
        # packs the int to a big-endian unsigned byte
        message = struct.pack(">B", message)

        self.client.sendall(message)

    # Sends a message for robot movement.
    # 
    # speed, turn: percents in range [-100, 100]
    # duration: milliseconds in range [0, 4,294,967,295]
    def send_movement(self, message):
        speed, turn, duration = message

        # packs the five parameters into an 6-byte binary string
        # ">bbIBB" = big-endian, signed byte, signed byte, unsigned 4-byte int
        message = struct.pack(">bbI", speed, turn, duration)
        
        self.client.sendall(message)
            
    def close(self):
        self.client.close()
        print("Connection closed.")
