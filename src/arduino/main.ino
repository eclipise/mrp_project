#include <SharpIR.h>

const bool DEBUG_MODE = true; // Enables additional printing over the serial connection
const int CLEAR_THRESHOLD = 40 // Distance at which an object is considered to be blocking an IR sensor (cm)

// Pin numbers for output; single digits are for PWM, all others are for direction
const int motorPinsLeft[6] = {5, 44, 46, 48, 50, 4};
const int motorPinsRight[6] = {3, 45, 47, 49, 51, 2};

const int PWM_MAX = 255; // Maximum value for motor PWM, equivalent to 0xFF

// Aliases the analog pins used by the IR sensors
#define IR_FL_Pin A1 // Front left
#define IR_FR_Pin A2 // Front right
#define IR_RR_Pin A3 // Rear right
#define IR_RL_Pin A4 // Rear left
#define IR_FC_Pin A5 // Front center

#define model SharpIR::GP2Y0A21YK0F // Model ID for the IR sensors, used internally in SharpIR

// Sets up the IR sensors
SharpIR IR_FR(model, IR_FR_Pin);
SharpIR IR_FL(model, IR_FL_Pin);
SharpIR IR_RR(model, IR_RR_Pin);
SharpIR IR_RL(model, IR_RL_Pin);
SharpIR IR_FC(model, IR_FC_Pin);

int fr_avg, fl_avg, rl_avg, rr_avg, fc_avg; // Moving average sensor data

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
    int ir_dist[5] = {0, 0, 0, 0, 0}; // Sum of 5 samples from each IR sensor [FR, FL, RR, RL, FC]

    // Sums 5 samples from each IR sensor (this should take between 100 and 150 ms)
    for (int i = 0; i < 5; i++) {
        ir_dist[0] += IR_FR.getDistance();
        ir_dist[1] += IR_FL.getDistance();
        ir_dist[2] += IR_RL.getDistance();
        ir_dist[3] += IR_RR.getDistance();
        ir_dist[4] += IR_FC.getDistance();
    }
    
    fr_avg = ir_dist[0] / 5;
    fl_avg = ir_dist[1] / 5;
    rl_avg = ir_dist[2] / 5;
    rr_avg = ir_dist[3] / 5;
    fc_avg = ir_dist[4] / 5;

    if (DEBUG_MODE) {
        // Returns the sensor data for debug
        Serial.print("Info: fr_dist: ");
        Serial.print(fr_avg);
        Serial.print("; fl_dist: ");
        Serial.print(fl_avg);
        Serial.print("; rl_dist: ");
        Serial.print(rl_avg);
        Serial.print("; rr_dist: ");
        Serial.print(rr_avg);
        Serial.print("; fc_dist: ");
        Serial.println(fc_avg);
    }
}

bool frontClear() {
    return fr_avg > CLEAR_THRESHOLD &&
           fl_avg > CLEAR_THRESHOLD &&
           fc_avg > CLEAR_THRESHOLD;
}

bool backClear() {
    return rl_avg > CLEAR_THRESHOLD &&
           rr_avg > CLEAR_THRESHOLD;
}

bool areaClear() {
    return fr_avg > CLEAR_THRESHOLD &&
           fl_avg > CLEAR_THRESHOLD &&
           rl_avg > CLEAR_THRESHOLD &&
           rr_avg > CLEAR_THRESHOLD &&
           fc_avg > CLEAR_THRESHOLD;
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
    
    if (DEBUG_MODE) {
        Serial.println("Status: Robot stopped");
    }
}

// Speed and turn should be in range [-100, 100], duration should be greater than 0 ms
int moveRobot(int speed, int turn, int duration) {
    if (DEBUG_MODE) {
        // Prints the command to the controller
        Serial.print("Status: Moving robot with command <speed: ");
        Serial.print(speed);
        Serial.print(", turn: ");
        Serial.print(turn);
        Serial.print(", duration: ");
        Serial.print(duration);
        Serial.println(">");
    }

    int leftSpeed, rightSpeed;
    speed /= 100; // Adjusts the ranges from [-100, 100] to [-1.00, 1.00]
    turn /= 100;

    // Return with failure code if the robot is not clear to perform the requested movement
    if (!checkClear(speed, turn)) {
        return 1; // Indicates failure to the caller
    }

    // Converts the input percent values to PWM values, mapping range [-100, 100]
    // to range [-255, 255]. i.e. 100 becomes 255, and -50 becomes -127.5.
    speed *= PWM_MAX;
    turn *= PWM_MAX;

    // If turn is positive, turn to the left by making left motors slower and right motors faster;
    // if turn is negative, turn to the right by making left motors faster and right motors slower.
    leftSpeed = speed - turn;
    rightSpeed = speed + turn;

    // Constrains the speed to range [-255, 255], the maximum value for PWM
    leftSpeed = constrain(leftSpeed, -PWM_MAX, PWM_MAX);
    rightSpeed = constrain(rightSpeed, -PWM_MAX, PWM_MAX);

    // Sends the speed to the motors
    analogWrite(motorPinsLeft[0], abs(leftSpeed));   // Pin 5
    analogWrite(motorPinsLeft[5], abs(leftSpeed));   // Pin 4
    analogWrite(motorPinsRight[0], abs(rightSpeed)); // Pin 3
    analogWrite(motorPinsRight[5], abs(rightSpeed)); // Pin 2

    if (DEBUG_MODE) {
        // Prints the speed values to the controller
        Serial.print("Info: Left motor speed: ");
        Serial.print(leftSpeed);
        Serial.print("; right motor speed: ");
        Serial.println(rightSpeed);
    }

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

    unsigned long startTime = millis(); // Logs the time the instruction started
    unsigned long currentTime;

    while (true) {
        currentTime = millis();

        // If at least the desired duration has elapsed, exit the loop
        if (currentTime - startTime >= duration) {
            break;
        }

        // If there is enough time left during the command to check the sensors
        if (abs(currentTime - startTime - duration) > 100){
            updateDistance();
        }
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

    // Updates IR sensor data constantly while idle to prevent lag when checking before movement
    updateDistance();

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

        // Errors if the speed is out of range
        if (speedInput > 100 || speedInput < -100) {
            Serial.println("Error: Speed must be in range [-100, 100]");
            error = true;
        }

        // Errors if the turn is out of range
        if (turnInput > 100 || turnInput < -100) {
            Serial.println("Error: Turn must be in range [-100, 100]");
            error = true;
        }

        // TODO: fix the duration maximum 
        // TODO: min duration may need to be constrained based on sensor check time
        // Errors if the duration is out of range
        if (durationInput <= 0) {
            Serial.println("Error: Duration must be in range [1, 32767] ms");
            error = true;
        }

        // On error, sends a sentinel message to the controller and returns without moving the robot
        if (error) {
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