import serial

class ArduinoController:
    def __init__(self) -> None:
        # establishes a serial connection to the Arduino, falls back to debug if this fails
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        except serial.serialutil.SerialException as e:
            print("Arduino not found:", e, sep="\n")
            self.arduino = None

    # Send a movement command and returns any messages sent back
    def send_movement(self, values: tuple[int, int, int]) -> list:
        # "speed turn duration"
        message = f"{values[0]} {values[1]} {values[2]}\n"
        
        # sends the message to the Arduino as a byte string
        self.arduino.write(message.encode())

        # Stores any messages the Arduino sends back, demarcated by newlines
        response = list()

        # receives the Arduino's response, in an unknown number of parts, until it sends a line
        # beginning with "END"
        while True:
            # if the connection is not idle, append received line to response
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode()
                response.append(line)
                
                # checks if the first three characters of the response are "END"
                if(line[:2] == "END"):
                    break

        return response
    