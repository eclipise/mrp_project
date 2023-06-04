import serial

class ArduinoController:
    def __init__(self) -> None:
        self.connected = False
        
        # Attempts to connect to the arduino
        self.connect_arduino

    # Sends a movement command and returns any messages sent back
    def send_movement(self, values: tuple[int, int, int]) -> list:
        # Errors if the Arduino is not connected
        if not self.arduino_connected:
            return list("Error: Arduino not connected", "END")
        
        # "speed turn duration"
        message = f"{values[0]} {values[1]} {values[2]}\n"
        
        # Sends the message to the Arduino as a byte string
        self.arduino.write(message.encode())

        # Stores any messages the Arduino sends back, demarcated by newlines
        response = list()

        # Receives the Arduino's response, in an unknown number of parts, until it sends a line
        # beginning with "END".
        while True:
            # If the connection is not idle, append received line to response
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode()
                response.append(line)
                
                # Breaks if the first three characters of the response are "END"
                if(line[:2] == "END"):
                    break

        return response
    
    # Establishes a serial connection to the Arduino, prints an error on fail
    def connect_arduino(self):
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.connected = True
        except serial.serialutil.SerialException as e:
            print("Arduino not found:", e)
            self.arduino = None
            self.connected = False
    