# Program Overview

The program’s GUI is implemented with PySimpleGUI in the `GUI` class of the `remote_app.py` file, which is also the main file to run for the remote application. When run, it creates a new object `gui` of the `GUI` class, which internally creates an object `client` of the `Client` class. On creation, `gui` generates a popup window prompting for the IP address of the server, erroring and re-prompting if the IP address is invalid or cannot be reached. Afterwards, it creates the main GUI window.

When the user presses one of the movement buttons on the GUI, the `gui` object calls the `send_movement` method of `gui.client`, which sends the appropriate movement command to the server using an HTTP POST request to the `/move` endpoint. 

The server is implemented in the `server.py` file using Flask, which is the main file running on the Jetson Nano. Upon receiving a POST to `/move`, the server validates the received JSON, checking that the three required fields, `move`, `turn`, and `duration` are present and in valid ranges (-100 to 100 for `move` and `turn`, greater than 0 for `duration`). If they are not valid, the server responds with code 400 and a list of the errors. If they are valid, the server passes them on to the Arduino using the `send_movement` method of an object `arduino_controller` of class `ArduinoController`, which is implemented in `arduino_controller.py`

The `main.iso` file, running on the Arduino, waits indefinitely for an incoming serial message. When it receives one and takes action, it responds with an arbitrary number of serial messages separated by newlines, ending communication with a sentinel message that begins with `END`. For example, its response may be: 

```
Status: Moving robot with command <speed: 100, turn: 0, duration: 200>\n
Info: Left motor speed: 255; right motor speed: 255\n
Status: Robot stopped\n
END: Success\n
```

Or, on failure:

```
Status: Moving robot with command <speed: -100, turn: 0, duration: 200>\n
Abort: Back is not clear\n
END: Failure\n
```

`ArduinoController` continues monitoring the serial connection and catching messages until it receives the sentinel message. 

On receiving a command in its `loop` function, the Arduino performs its own checks on the values (which is redundant but good practice). If the values fail this check, the Arduino responds with the errors and waits for another message. If they pass, it calls its own `moveRobot` function, which checks if the robot is clear to move and moves it if it is. The duration of the movement command is handled by a loop that iterates every 10 ms, stopping the robot if it is no longer clear to move or if the current time is at least `duration` ms from the time the movement started. If the movement completed uninterrupted, `moveRobot` returns 0, else it returns 1 (which `loop` uses to determine whether to send "END: Success" or "END: Failure").

The logic that checks if the robot is clear to move is in `main.ino`'s `frontClear`, `backClear`, and `areaClear` functions, which each update the data from the IR sensors before executing. `frontClear` checks if the front left and right sensors are clear to 30 cm, and if the front center sensor is clear to 20 cm. `backClear` checks if the rear left and right sensors are clear to 30 cm. `areaClear` is used for turning and checks if all sensors are clear to 10 cm. These three functions are called from the `checkClear` function, which takes `speed` and `turn` values and performs the appropriate check based on them. 

After the Arduino finishes and `ArduinoController` receives the sentinel message, it returns a list of the messages to the server, which sends them back to the client using an HTTP response with code 200. The client passes them unchanged to the GUI, which at present prints them to the console.