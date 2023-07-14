import json
import websocket
import rel
from flask import Flask, request, Response

app = Flask(__name__)

def on_open(ws):
    print("ws open")

    # Advertise cmd_vel to ROS
    msg = {
        "op": "advertise",
        "topic": "/cmd_vel",
        "type": "geometry_msgs/Twist"
    }

    ws.send(json.dumps(msg))

def on_close(ws, close_status_code, close_msg):
    print("ws closed:", close_status_code, close_msg)

def on_error(ws, error):
    print("ws error:", error)

def on_message(ws, msg):
    print(msg)

ws = websocket.WebSocketApp("ws://localhost:8080", on_open=on_open, on_message=on_message, on_error=on_error, on_close=on_close)

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

    message = {
        "op": "publish",
        "topic": "/cmd_vel",
        "msg": {
            "linear": {
                "x": values[0]/100,
                "y": 0.0,
                "z": 0.0
            },
            "angular": {
                "x": 0.0,
                "y": 0.0,
                "z": values[1]/100
            }
        }
    }

    ws.send(json.dumps(message))

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
    # Automatically reconnects after 5 seconds if the connection drops unexpectedly
    ws.run_forever(dispatcher=rel, ping_interval=5)
    # rel.signal(2, rel.abort)  # Keyboard Interrupt
    # rel.dispatch()

    app.run(debug=False, host="0.0.0.0")
