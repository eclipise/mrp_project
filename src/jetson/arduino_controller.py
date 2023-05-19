# main program running on Jetson Nano

import serial
from server import Server

def main():
    # # opens serial connection to the Arduino
    # arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    # initializes socket server
    port = 4500 # arbitrary port
    server = Server(port)
    server.open_server()

    print(server.recv_movement())

    server.close()


if __name__ == "__main__":
    main()