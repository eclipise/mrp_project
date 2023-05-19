# contains a server implementation for the Jetson Nano

import socket
import sys

# initializes server on instantiation, must then be opened with open_server method
class Server:
    def __init__(self, port):
        self.port = port,
        self.server = self.init_server(),
        self.client, 
        self.client_addr

    def get_host_ip(self):
        return socket.gethostbyname(socket.gethostname())

    def get_client_addr(self):
        return self.client_addr

    def init_server(self):
        try:
            # initializes a network socket
            print("Creating socket...")
            host = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("Success.")
            
            # binds the socket to a port
            print(f"Binding socket to port {self.port}...")
            host.bind(('', self.port))
            print("Success.")
        except socket.error as err:
            print("Error:", err)
            sys.exit(1)

    # makes the server wait for an incoming connection
    def open_server(self):
        host_ip = self.get_host_ip()

        # starts listening for incoming connections
        print(f"Waiting for connection at {host_ip}:{self.port}...")
        self.server.listen()
        
        # waits for a connection request
        self.client, self.client_addr = self.server.accept()
        print(f"Connected to {self.client_addr}.")
    
    def send(self, message):
        if self.client is None:
            print("Error, attempted to send to uninitialized client.")
            sys.exit(1)

        message = message.encode()
        message_length = len(message)
        bytes_sent = 0

        # continues sending until the entire message has been sent
        while bytes_sent < message_length:
            bytes_sent += self.client.send(message.encode())

    def recv(self, len):
        return self.client.recv(len).decode

    def close_connection(self):
        self.client.close()
