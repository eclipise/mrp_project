from flask import Flask, request, Response

app = Flask(__name__)

@app.get("/")
def base_handler():
    return Response(status=200)

@app.post("/move")
def move_handler():
    message = request.get_json()

    values = validate_movement(message)

    # If there are no errors, remove that from the tuple; else send the errors
    if len(values[2]) == 0:
        values = (values[0], values[1])
    else:
        return {"errors": values[2]}, 400 # 400: bad request

    # Sends the Arduino's response to the client
    return Response(status=200)

def validate_movement(message: dict) -> tuple:
    # Checks if the required keys are present
    speed_present = "speed" in message.keys()
    turn_present = "turn" in message.keys()
    
    # #######################################################################
    # If a key is not present, appends the relevant error to return message.
    # If a key is present, checks if its value is in the valid range and 
    # appends the relevant error to the return message if it is not.

    # Remains empty if there are no errors
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

    # #######################################################################

    # Returns values as ints and either an error string or an empty string on success
    return (speed, turn, errors)


if __name__ == "__main__":    
    app.run(debug=False, host="0.0.0.0")
