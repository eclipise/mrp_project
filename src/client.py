# file to run on controlling computer
# contains the client that connects to the Jetson Nano's server

import socket
import sys

# initializes a network socket
print("Creating socket...")

try:
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Success.")
except socket.error as err:
    print("Error:", err)
    sys.exit(1)

# accepts the full ip and port of the host from the user,
# splits into IP and port across ':'
host_addr = input("Enter the address of the host (X.X.X.X:port): ").split(':')
host_ip = host_addr[0]
host_port = int(host_addr[1])

try:
    # connects to the host at host_ip:host_port
    print(f"Connecting to {host_ip}:{host_port}...")
    client.connect((host_ip, host_port))
    print("Connected.")
except socket.error as err:
    print("Error:", err)
    sys.exit(1)


