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
    # speed and turn are percents in range [-100, 100], 
    # duration is milliseconds in range [0, 4,294,967,295]
    def send_movement(self, speed, turn, duration):
        # packs the three parameters into a 6-byte binary string
        # ">bbI" = big-endian, signed byte, signed byte, unsigned 4-byte int
        message = struct.pack(">bbI", speed, turn, duration)
        
        self.client.send(message)
            
    def close(self):
        self.client.close()
        print("Connection closed.")




