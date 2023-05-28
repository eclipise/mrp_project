#include <SharpIR.h>

// Pin numbers for output; single digits are for PWM, all others are for direction
const int motorPinsLeft[6] = {5, 44, 46, 48, 50, 4};
const int motorPinsRight[6] = {3, 45, 47, 49, 51, 2};

const int PWM_MAX = 255; // Maximum value for motor PWM, equivalent to 0xFF

// int theTorque = 1.0625; // Conversion of how many bits will result in 1rpm

// Aliases the analog pins A1 and A2 with the names of their connected IR sensors
#define IR1 A1
#define IR2 A2

// TODO
#define model 1080

// Sets up the IR sensors
SharpIR SharpIR1(IR1, model);
SharpIR SharpIR2(IR2, model);

int dis[2] = {0, 0}; // Array for IR sensor data

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

void getDistance(int *dis) {
    unsigned long pepe1 = millis(); // takes the time before the loop on the library begins

    int dis1 = SharpIR1.distance(); // this returns the distance to the object you're measuring
    int dis2 = SharpIR2.distance();

    dis[0] = dis1; // Writing value of IR sensor onto location 0 of matrix
    dis[1] = dis2; // Writing value of IR sensor onto location 1 of matrix
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

    Serial.println("STOP");
}

// Speed and turn should be in range [-1.0, 1.0], duration should be greater than 0
void moveRobot(float speed, float turn, int duration) {
    int leftSpeed, rightSpeed;

    // Prints a status message to the controller
    Serial.print("Info: Moving robot with command {speed: ");
    Serial.print(speed);
    Serial.print(", turn: ");
    Serial.print(turn);
    Serial.print(", duration: ");
    Serial.print(duration);
    Serial.println("}");

    getDistance(dis);

    if (dis[0] < 20 || dis[1] < 30) {
        stopRobot();
        return;
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

    Serial.println("Speed for the left motors");
    Serial.println(leftSpeed);
    Serial.println("Speed for the right motors");
    Serial.println(rightSpeed);
    // Serial.println("Speed for the left motors");
    // Serial.println(theTorque);

    // Sets the int pins for the left motors
    if (leftSpeed > 0) {
        // Forward
        digitalWrite(motorPinsLeft[1], 1); // Pin 44, int1
        digitalWrite(motorPinsLeft[2], 0); // Pin 46, int2
        digitalWrite(motorPinsLeft[3], 1); // Pin 48, int4
        digitalWrite(motorPinsLeft[4], 0); // Pin 50, int3
        Serial.println("------------Forward Left");
    } else {
        // Backward
        digitalWrite(motorPinsLeft[1], 0);
        digitalWrite(motorPinsLeft[2], 1);
        digitalWrite(motorPinsLeft[3], 0);
        digitalWrite(motorPinsLeft[4], 1);
        Serial.println("------------Backward Left");
    }

    // Sets the int pins for the right motors
    if (rightSpeed > 0) {
        // Forward
        digitalWrite(motorPinsRight[1], 1); // Pin 45, int1
        digitalWrite(motorPinsRight[2], 0); // Pin 47, int2
        digitalWrite(motorPinsRight[3], 0); // Pin 49, int4
        digitalWrite(motorPinsRight[4], 1); // Pin 51, int3
        Serial.println("------------Forward Right");
    } else {
        // Backwards
        digitalWrite(motorPinsRight[1], 0);
        digitalWrite(motorPinsRight[2], 1);
        digitalWrite(motorPinsRight[3], 1);
        digitalWrite(motorPinsRight[4], 0);
        Serial.println("------------Backward Right");
    }

    // Waits for the duration of the instruction, then stops the robot
    delay(duration);
    stopRobot();
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
            Serial.println("END: failure");
            return;
        }

        moveRobot(speedInput, turnInput, durationInput);

        // Informs the controller of successful movement
        Serial.println("END: success");
    }
}