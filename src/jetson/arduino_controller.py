# main program running on Jetson Nano

import serial
from time import sleep
from server import Server

def main():
    # # opens serial connection to the Arduino
    # arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    # initializes socket server
    port = 4500 # arbitrary port
    server = Server(port)
    server.open_server()

    while True:
        if server.recv_control() > 0:
            break

        message = server.recv_movement()
        print(message)
        
        # wait for the duration of the instruction 
        sleep(message[2]/1000)
        
    server.close()

if __name__ == "__main__":
    main()
