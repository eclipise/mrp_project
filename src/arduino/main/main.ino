#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <SharpIR.h>

// Handles integration with ROS
ros::NodeHandle nh;

/* ---------------------------- Program constants --------------------------- */

// Enables additional printing over the serial connection
const bool DEBUG_MODE = false;

// Distance in cm at which an IR sensor is considered blocked when moving forward or backwards
const int LINEAR_CLEAR_THRESHOLD = -1;
// Distance in cm at which an IR sensor is considered blocked when turning
const int TURN_CLEAR_THRESHOLD = -1;

// Time in milliseconds to continue the last command before stopping, in absence of a new command
const unsigned COMMAND_TIMEOUT = 200;

// Values below PWM_MIN will be treated as 0
const int PWM_MIN = 24; // one less than 10% power
// Values above PWM_MAX will be reduced to PWM_MAX
const int PWM_MAX = 50; // 20% power
// PWM value used when turning in place (+/- for each side, depending on direction)
const int PWM_TURN = 75; // 30% power

/* ---------------------------- Arduino pin setup --------------------------- */

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

// Model ID for the IR sensors, used internally in SharpIR
#define model SharpIR::GP2Y0A21YK0F

// Sets up the IR sensors, second parameter is the pin they're connected to
SharpIR IR_FL(model, A1); // Front left
SharpIR IR_FR(model, A2); // Front right
SharpIR IR_RR(model, A3); // Rear right
SharpIR IR_RL(model, A4); // Rear left
SharpIR IR_FC(model, A5); // Front center

/* ------------------------------ Program data ------------------------------ */

// These values are from -255 to 255, indicating required speed and direction
int pwmLeftReq = 0;
int pwmRightReq = 0;

// Timestamp in milliseconds of the last command
unsigned long lastCommandTime = 0;

// IR sensor data
int fr_dist, fl_dist, rl_dist, rr_dist, fc_dist;

/* -------------------------------------------------------------------------- */

void updateDistance() {
    fr_dist = IR_FR.getDistance();
    fl_dist = IR_FL.getDistance();
    rl_dist = IR_RL.getDistance();
    rr_dist = IR_RR.getDistance();
    fc_dist = IR_FC.getDistance();

    if (DEBUG_MODE) {
        // Returns the sensor data for debug
        Serial.print("Info: fr_dist: ");
        Serial.print(fr_dist);
        Serial.print("; fl_dist: ");
        Serial.print(fl_dist);
        Serial.print("; rl_dist: ");
        Serial.print(rl_dist);
        Serial.print("; rr_dist: ");
        Serial.print(rr_dist);
        Serial.print("; fc_dist: ");
        Serial.println(fc_dist);
    }
}

bool frontClear() {
    return fr_dist > LINEAR_CLEAR_THRESHOLD &&
           fl_dist > LINEAR_CLEAR_THRESHOLD &&
           fc_dist > LINEAR_CLEAR_THRESHOLD;
}

bool backClear() {
    return rl_dist > LINEAR_CLEAR_THRESHOLD &&
           rr_dist > LINEAR_CLEAR_THRESHOLD;
}

bool areaClear() {
    return fr_dist > TURN_CLEAR_THRESHOLD &&
           fl_dist > TURN_CLEAR_THRESHOLD &&
           rl_dist > TURN_CLEAR_THRESHOLD &&
           rr_dist > TURN_CLEAR_THRESHOLD &&
           fc_dist > TURN_CLEAR_THRESHOLD;
}

bool checkClear() {
    // If not turning
    if (pwmLeftReq == pwmRightReq) {
        if (pwmLeftReq > 0 && !frontClear()) {
            // If moving forward and front is not clear
            // Serial.println("Abort: Front is not clear");
            return false;
        } else if (pwmLeftReq < 0 && !backClear()) {
            // If moving backwards and rear is not clear
            // Serial.println("Abort: Back is not clear");
            return false;
        }

        // Else PWM is 0 and the robot isn't moving
        return true;
    } else if (!areaClear()) {
        // If turning and area is not clear
        // Serial.println("Abort: Area is not clear");
        return false;
    }

    return true;
}

// Updates the required PWM values every time a new ROS command comes in, but does not
// update the values sent to the motors.
void calc_pwm(const geometry_msgs::Twist &cmdVel) {
    msg.data = "calculating pwm";
    chatter.publish(&msg);

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
    msg.data = "setting pwm";
    chatter.publish(&msg);
    
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

std_msgs::String msg;
ros::Publisher chatter("chatter", &msg);

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
    nh.initNode();
    nh.subscribe(subCmdVel);
}

// Main loop, runs repeatedly while Arduino is on
void loop() {
    // Handles incoming ROS messages (which calls calc_pwm per the subCmdVel subscriber)
    nh.spinOnce();

    // Stops the robot if it has been more than COMMAND_TIMEOUT ms since the last instruction
    if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
        msg.data = "timeout";
        chatter.publish(&msg);

        pwmLeftReq = 0;
        pwmRightReq = 0;
    } else {
        // Checks if the robot is clear to move and stops it if it is not

        updateDistance();

        if (!checkClear()) {
            msg.data = "blocked";
            chatter.publish(&msg);

            pwmLeftReq = 0;
            pwmRightReq = 0;
        }
    }

    set_pwm();
}