import serial

# Prompt the user for input parameters
param1 = int(input("Enter Speed (0 to 100): "))
param2 = int(input("Enter Turn degree (0 to 100): "))
param3 = int(input("Enter time of execution (1000 = 1sec): "))

# Open a serial connection to the Arduino
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Send the parameters to the Arduino
message = "{} {} {}\n".format(param1, param2, param3)
ser.write(message.encode())

# Wait for the Arduino to respond
while ser.in_waiting == 0:
  pass

# Read the response from the Arduino
response = ser.readline().decode().rstrip()

# Print the response
print("Received response: {}".format(response))

# Close the serial connection
ser.close()

