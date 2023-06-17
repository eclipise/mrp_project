#include <SharpIR.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>

const bool DEBUG_MODE = true;   // Enables additional printing over the serial connection
const int CLEAR_THRESHOLD = 30; // Distance at which an object is considered to be blocking an IR sensor (cm)

ros::NodeHandle nh;

unsigned long lastCommandTime = 0; // Timestamp in milliseconds of the last command

// Time in milliseconds to continue the last command before stopping, in absence of a new command
const unsigned COMMAND_TIMEOUT = 200;

// These values are from -255 to 255, indicating required speed and direction
int pwmLeftReq = 0;
int pwmRightReq = 0;

// PWM value used when turning in place (+- for each side, depending on direction)
const int PWM_TURN = 80;

const int PWM_MIN = 10;  // Values below PWM_MIN will be treated as 0
const int PWM_MAX = 255; // Values above PWM_MAX will be reduced to PWM_MAX

// Pins for connection to the left motor driver
const int L_ENA = 4;
const int L_ENB = 5;
const int L_INT1 = 44;
const int L_INT2 = 46;
const int L_INT3 = 48;
const int L_INT4 = 50;

// Pins for connection to the right motor driver
const int R_ENA = 2;
const int R_ENB = 3;
const int R_INT1 = 45;
const int R_INT2 = 47;
const int R_INT3 = 49;
const int R_INT4 = 51;

#define model SharpIR::GP2Y0A21YK0F // Model ID for the IR sensors, used internally in SharpIR

// Sets up the IR sensors, second parameter is the pin they're connected to
SharpIR IR_FL(model, A1); // Front left
SharpIR IR_FR(model, A2); // Front right
SharpIR IR_RR(model, A3); // Rear right
SharpIR IR_RL(model, A4); // Rear left
SharpIR IR_FC(model, A5); // Front center

int fr_avg, fl_avg, rl_avg, rr_avg, fc_avg; // Average IR sensor data; use this instead of the raw

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

// Updates the required PWM values every time a new ROS command comes in, but does not
// update the values sent to the motors.
void calc_pwm(const geometry_msgs::Twist &cmdVel) {
    lastCommandTime = millis();

    // For now, this interprets incoming cmdVel messages with m/s and rad/s values as
    // percents of max PWM. Full forward is 1.00 m/s, reverse is -1.00 m/s, full left turn
    // is 1.00 rad/s, full right turn is -1.00 rad/s, etc. Uses linear.x and angular.z.

    pwmLeftReq = cmdVel.linear.x * 255;
    pwmRightReq = cmdVel.linear.x * 255;

    // If commanded to turn
    if (cmdVel.angular.z != 0) {
        if (cmdVel.angular.z > 0) {
            // Left turn
            pwmLeftReq = -PWM_TURN;
            pwmRightReq = PWM_TURN;
        } else {
            // Right turn
            pwmLeftReq = PWM_TURN;
            pwmRightReq = -PWM_TURN;
        }
    }

    /* ------------- Constrains PWM values using PWM_MIN and PWM_MAX ------------ */
    if (abs(pwmLeftReq) < PWM_MIN) {
        pwmLeftReq = 0;
    } else if (abs(pwmLeftReq) > PWM_MIN) {
        pwmLeftReq = PWM_MAX;
    }

    if (abs(pwmRightReq) < PWM_MIN) {
        pwmRightReq = 0;
    } else if (abs(pwmRightReq) > PWM_MIN) {
        pwmRightReq = PWM_MAX;
    }
    /* -------------------------------------------------------------------------- */
}

void set_pwm() {
    /* -------------------- Sets the direction of the motors -------------------- */
    if (pwmLeftReq > 0) {
        // Left forward
        digitalWrite(L_INT1, 1);
        digitalWrite(L_INT2, 0);
        digitalWrite(L_INT3, 1);
        digitalWrite(L_INT4, 0);
    } else if (pwmLeftReq < 0) {
        // Left reverse
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 1);
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 1);
    } else {
        // Left stop
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 0);
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 0);
    }

    if (pwmRightReq > 0) {
        // Right forward
        digitalWrite(R_INT1, 1);
        digitalWrite(R_INT2, 0);
        digitalWrite(R_INT3, 1);
        digitalWrite(R_INT4, 0);
    } else if (pwmRightReq < 0) {
        // Right reverse
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 1);
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 1);
    } else {
        // Right stop
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 0);
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 0);
    }
    /* -------------------------------------------------------------------------- */

    // Writes the PWM values (which should already be constrained)
    analogWrite(L_ENA, abs(pwmLeftReq));
    analogWrite(L_ENB, abs(pwmLeftReq));
    analogWrite(R_ENA, abs(pwmRightReq));
    analogWrite(R_ENB, abs(pwmRightReq));
}

// Sets a ROS subscriber to handle velocity commands with the calc_pwm function
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm);

// Runs once on Arduino startup and is used for initialization
void setup() {
    // Sets all motor control pins to output mode
    pinMode(L_ENA, OUTPUT);
    pinMode(L_ENB, OUTPUT);
    pinMode(L_INT1, OUTPUT);
    pinMode(L_INT2, OUTPUT);
    pinMode(L_INT3, OUTPUT);
    pinMode(L_INT4, OUTPUT);
    pinMode(R_ENA, OUTPUT);
    pinMode(R_ENB, OUTPUT);
    pinMode(R_INT1, OUTPUT);
    pinMode(R_INT2, OUTPUT);
    pinMode(R_INT3, OUTPUT);
    pinMode(R_INT4, OUTPUT);

    // Turns off the motors by default
    digitalWrite(L_INT1, 0);
    digitalWrite(L_INT2, 0);
    digitalWrite(L_INT3, 0);
    digitalWrite(L_INT4, 0);
    digitalWrite(R_INT1, 0);
    digitalWrite(R_INT2, 0);
    digitalWrite(R_INT3, 0);
    digitalWrite(R_INT4, 0);

    // Sets the initial PWM values to 0
    analogWrite(L_ENA, 0);
    analogWrite(L_ENB, 0);
    analogWrite(R_ENA, 0);
    analogWrite(R_ENB, 0);

    // ROS setup
    nh.getHardware()->setBaud(9600);
    nh.initNode();
    nh.subscribe(subCmdVel);
}

// Main loop, runs repeatedly while Arduino is on
void loop() {
    // Handles incoming ROS messages (which calls calc_pwm per the subCmdVel subscriber)
    nh.spinOnce();

    // Stops the robot if it has been more than COMMAND_TIMEOUT ms since the last instruction
    if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
        pwmLeftReq = 0;
        pwmRightReq = 0;
    }

    set_pwm();
}