#include <SharpIR.h>

// Pin numbers for output; single digits are for PWM, all others are for direction
const int motorPinsLeft[6] = {5, 44, 46, 48, 50, 4};
const int motorPinsRight[6] = {3, 45, 47, 49, 51, 2};

const int PWM_MAX = 255; // Maximum value for motor PWM, equivalent to 0xFF

// int theTorque = 1.0625; // PWM value equal to 1rpm

// Aliases the analog pins used by the IR sensors
#define IR0_Pin A1 // Front left
#define IR1_Pin A2 // Front right
#define IR2_Pin A3 // Rear right
#define IR3_Pin A4 // Rear left
#define IR4_Pin A5 // Front center

#define model 1080 // Model ID for the IR sensors, used internally in SharpIR

// Sets up the IR sensors
SharpIR IR0(IR0_Pin, model);
SharpIR IR1(IR1_Pin, model);
SharpIR IR2(IR2_Pin, model);
SharpIR IR3(IR3_Pin, model);
SharpIR IR4(IR4_Pin, model);

int distance[5] = {0, 0, 0, 0, 0}; // Stores IR sensor data in cm {FL, FR, RR, RL, FC}

// Runs once on Arduino startup and is used for initialization
void setup() {
    // Sets all pins to output mode
    for (int i = 0; i < 6; i++) {
        pinMode(motorPinsLeft[i], OUTPUT);
        pinMode(motorPinsRight[i], OUTPUT);
    }

    // Starts serial communication with the Arduino using baud rate 9600
    Serial.begin(9600);
}

// Refreshes the distance array
void updateDistance() {
    // Gets the output of each IR sensor in cm
    distance[0] = IR0.distance();
    distance[1] = IR1.distance();
    distance[2] = IR2.distance();
    distance[3] = IR3.distance();
    distance[4] = IR4.distance();
}

bool frontClear() {
    updateDistance();

    // True if front left and right are clear to 30 cm, front center is clear to 20 cm
    return distance[0] > 30 &&
           distance[1] > 30 &&
           distance[4] > 20;
}

bool backClear() {
    updateDistance();

    // True if rear left and right are clear to 30 cm
    return distance[2] > 30 &&
           distance[3] > 30;
}

// Check if all sensors are clear at a shorter range, used for turning in place
bool areaClear() {
    updateDistance();

    // True if all sensors are clear to 10 cm
    return distance[0] > 10 &&
           distance[1] > 10 &&
           distance[2] > 10 &&
           distance[3] > 10 &&
           distance[4] > 10;
}

bool checkClear(int speed, int turn) {
    // Aborts if robot is told to move forward and is not clear to do so
    if (speed > 0 && !frontClear()) {
        Serial.println("Abort: Front is not clear");
        return false;
    }

    // Aborts if robot is told to move backwards and is not clear to do so
    if (speed < 0 && !backClear()) {
        Serial.println("Abort: Back is not clear");
        return false;
    }

    // Aborts if robot is told to turn in place and the immediate area is not clear
    if (turn != 0 && !areaClear()) {
        Serial.println("Abort: Area is not clear");
        return false;
    }

    return true;
}

void stopRobot() {
    digitalWrite(motorPinsLeft[1], 0);  // Pin 44, int1
    digitalWrite(motorPinsLeft[2], 0);  // Pin 46, int2
    digitalWrite(motorPinsLeft[3], 0);  // Pin 48, int4
    digitalWrite(motorPinsLeft[4], 0);  // Pin 50, int3
    digitalWrite(motorPinsRight[1], 0); // Pin 45, int1
    digitalWrite(motorPinsRight[2], 0); // Pin 47, int2
    digitalWrite(motorPinsRight[3], 0); // Pin 49, int4
    digitalWrite(motorPinsRight[4], 0); // Pin 51, int3

    Serial.println("Status: Robot stopped");
}

// Speed and turn should be in range [-1.0, 1.0], duration should be greater than 0 ms
int moveRobot(float speed, float turn, int duration) {
    int leftSpeed, rightSpeed;

    // Prints the command to the controller
    Serial.print("Status: Moving robot with command {speed: ");
    Serial.print(speed);
    Serial.print(", turn: ");
    Serial.print(turn);
    Serial.print(", duration: ");
    Serial.print(duration);
    Serial.println("}");

    // Return with failure code if the robot is not clear to perform the requested movement
    if (!checkClear(speed, turn)) {
        return 1; // Indicates failure to the caller
    }

    // Converts the input percent values to PWM values, mapping range [-1.0, 1.0]
    // to range [-255, 255]. i.e. 1.0 becomes 255, and -0.5 becomes -127.5.
    speed *= PWM_MAX;
    turn *= PWM_MAX;

    // theTorque *= speed; // Math conversion of the bits to rpm for the motors

    // If turn is positive, turn to the left by making left motors slower and right motors faster;
    // if turn is negative, turn to the right by making left motors faster and right motors slower.
    leftSpeed = int(speed - turn);
    rightSpeed = int(speed + turn);

    // Constrains the speed to range [-255, 255], the maximum value for PWM
    leftSpeed = constrain(leftSpeed, -PWM_MAX, PWM_MAX);
    rightSpeed = constrain(rightSpeed, -PWM_MAX, PWM_MAX);

    // Sends the speed to the motors
    analogWrite(motorPinsLeft[0], abs(leftSpeed));   // Pin 5
    analogWrite(motorPinsLeft[5], abs(leftSpeed));   // Pin 4
    analogWrite(motorPinsRight[0], abs(rightSpeed)); // Pin 3
    analogWrite(motorPinsRight[5], abs(rightSpeed)); // Pin 2

    // Prints the speed values to the controller
    Serial.print("Info: Left motor speed: ");
    Serial.print(leftSpeed);
    Serial.print("; right motor speed: ");
    Serial.println(rightSpeed);

    // Sets the int pins for the left motors
    if (leftSpeed > 0) {
        // Forward
        digitalWrite(motorPinsLeft[1], 1); // Pin 44, int1
        digitalWrite(motorPinsLeft[2], 0); // Pin 46, int2
        digitalWrite(motorPinsLeft[3], 1); // Pin 48, int4
        digitalWrite(motorPinsLeft[4], 0); // Pin 50, int3
    } else {
        // Backward
        digitalWrite(motorPinsLeft[1], 0);
        digitalWrite(motorPinsLeft[2], 1);
        digitalWrite(motorPinsLeft[3], 0);
        digitalWrite(motorPinsLeft[4], 1);
    }

    // Sets the int pins for the right motors
    if (rightSpeed > 0) {
        // Forward
        digitalWrite(motorPinsRight[1], 1); // Pin 45, int1
        digitalWrite(motorPinsRight[2], 0); // Pin 47, int2
        digitalWrite(motorPinsRight[3], 0); // Pin 49, int4
        digitalWrite(motorPinsRight[4], 1); // Pin 51, int3
    } else {
        // Backward
        digitalWrite(motorPinsRight[1], 0);
        digitalWrite(motorPinsRight[2], 1);
        digitalWrite(motorPinsRight[3], 1);
        digitalWrite(motorPinsRight[4], 0);
    }

    // Waits for the duration of the instruction, then stops the robot

    // Logs the time the instruction started
    unsigned long startTime = millis();
    unsigned long currentTime;

    // Checks if the robot is still clear to move every 10 ms as the instruction runs
    while (true) {
        currentTime = millis();

        // If at least the desired duration has elapsed, exit the loop
        if (currentTime - startTime >= duration) {
            break;
        }

        // Stops the robot if it is no longer clear to move
        if (!checkClear(speed, turn)) {
            stopRobot();
            return 1; // Indicates failure to the caller
        }

        // Waits for 10 milliseconds before checking again
        delay(10);
    }

    stopRobot();
    return 0; // Indicates success to the caller
}

// Main loop, runs repeatedly while Arduino is on
void loop() {
    // Variables for input from the controlling program
    int speedInput, turnInput, durationInput;

    // Tracks if there has been a fatal error
    bool error = false;

    if (Serial.available() > 0) {
        // Reads the incoming message
        String message = Serial.readStringUntil('\n');

        // Parses the parameters from the message; result stores the number of parameters found
        int result = sscanf(message.c_str(), "%d %d %d", &speedInput, &turnInput, &durationInput);

        // Errors if there were fewer than three numbers in the message
        if (result < 3) {
            Serial.println("Error: Too few parameters");
            error = true;
        }

        // Errors if the duration is less than or equal to 0
        if (durationInput <= 0) {
            Serial.println("Error: Duration must be greater than 0");
            error = true;
        }

        // Errors if the speed is out of range
        if (speedInput > 1.0 || speedInput < -1.0) {
            Serial.println("Error: Speed must be in range [-1.0, 1.0]");
            error = true;
        }

        // Errors if the turn is out of range
        if (turnInput > 1.0 || turnInput < -1.0) {
            Serial.println("Error: Turn must be in range [-1.0, 1.0]");
            error = true;
        }

        // On error, sends a sentinel message to the controller and returns without moving the robot
        if error {
            Serial.println("END: Errors");
            return;
        }

        result = moveRobot(speedInput, turnInput, durationInput);
        if (result > 0) {
            Serial.println("END: Failure");
        } else {
            Serial.println("END: Success");
        }
    }
}