from flask import Flask, request, Response, jsonify
from arduino_controller import ArduinoController

app = Flask(__name__)
# arduino_controller = ArduinoController()

@app.get("/")
def base_handler():
    return Response(status=200)

@app.post("/move")
def move_handler():
    message = request.get_json()

    values = validate_movement(message)

    # if there are no errors, remove that from the tuple; else send the errors
    if len(values[3]) == 0:
        values = (values[0], values[1], values[2])
    else:
        return {"errors": values[3]}, 400
    
    # sends the movement command to the Arduino and gets its response
    # arduino_response = arduino_controller.send_movement(values)

    # sends the Arduino's response to the client
    # return {"arduino response": arduino_response}, 200

    print(values)

    return {"arduino response": "Sample response success"}, 200

@app.get("/status")
def status_handler():
    status = {
        "battery": 95
    }

    return status

def validate_movement(message: dict) -> tuple[int, int, int, list]:
    # checks if the required keys are present
    speed_present = "speed" in message.keys()
    turn_present = "turn" in message.keys()
    duration_present = "duration" in message.keys()
    
    # #######################################################################
    # If a key is not present, appends the relevant error to return message.
    # If a key is present, checks if its value in the valid range and 
    # appends the relevant error to the return message if it is not.

    # remains empty if there are no errors
    errors = list()

    if speed_present:
        speed = int(message["speed"])
        
        if not 100 >= speed >= -100:
            errors.append(f"Speed <{speed}> is out of range [-100, 100]")
    else:
        speed = 0 # dummy value for return
        errors.append("Missing key: 'speed'")
    

    if turn_present:
        turn = int(message["turn"])
        
        if not 100 >= turn >= -100:
            errors.append(f"Turn <{turn}> is out of range [-100, 100]")
    else:
        turn = 0 # dummy value for return
        errors.append("Missing key: 'turn'")
    
    
    if duration_present:
        duration = int(message["duration"])
        
        if not duration > 0:
            errors.append(f"Duration <{duration}> is out of range (inf, 0)")
    else:
        duration = 0 # dummy value for return
        errors.append("Missing key: 'duration'")

    # #######################################################################

    # returns values as ints and either an error string or an empty string on success
    return (speed, turn, duration, errors)


if __name__ == "__main__":
    app.run(debug=True)
