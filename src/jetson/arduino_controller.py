import serial

class ArduinoController:
    def __init__(self) -> None:
        # establishes a serial connection to the Arduino
        self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    def send_movement(self, values: tuple[int, int, int]) -> str:
        # "speed turn duration"
        message = f"{values[0]} {values[1]} {values[2]}"
        
        # sends the message to the Arduino as a byte string
        self.arduino.write(message.encode())

        # waits for the Arduino to send a response
        while self.arduino.in_waiting == 0:
            pass

        # reads a string from the Arduino
        response = self.arduino.readline().decode()

        return response
    