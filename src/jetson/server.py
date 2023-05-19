# contains a server implementation for the Jetson Nano

import socket
import struct
import sys

# initializes server socket on instantiation,
# must then be opened with open_server method
class Server:
    def __init__(self, port):
        self.port = port
        self.server = self.init_server()
        self.client = None
        self.client_addr = None

    def get_client_addr(self):
        return self.client_addr

    def init_server(self):
        try:
            # initializes a network socket
            print("Creating socket...")
            host = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("success.")
            
            # binds the socket to a port
            print(f"Binding socket to port {self.port}...")
            host.bind(('', self.port))
            print("success.")
        except socket.error as err:
            print("error:", err)
            host.close()
            sys.exit(1)

        return host

    # makes the server wait for an incoming connection
    def open_server(self):
        # starts listening for incoming connections
        print(f"Waiting for connection...")
        self.server.listen()
        
        # waits for a connection request
        self.client, self.client_addr = self.server.accept()
        print(f"connected to {self.client_addr}.")
    
    def send(self, message):
        if self.client is None:
            print("Error, attempted to send to uninitialized client.")
            self.server.close()
            sys.exit(1)

        self.client.send(message.encode())

    # receives flags for overall program control (i.e. exiting)
    def recv_control(self):
        # receives 1 byte
        message = self.client.recv(1)

        # unpack produces a tuple with one element, so this returns only that element
        return struct.unpack(">B", message)[0]

    # receives movement instructions
    def recv_movement(self):
        # receives 6 bytes
        message = self.client.recv(6)

        # unpacks the bytes according to the format string:
        # ">bbI" = big-endian, signed byte, signed byte, unsigned 4-byte int
        speed, turn, duration = struct.unpack(">bbI", message)

        return (speed, turn, duration)

    def close(self):
        self.client.close()
        print("Connection closed.")
