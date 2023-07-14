#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <SharpIR.h>

// Base component necessary to make this program a ROS node
ros::NodeHandle nh;

/* ---------------------------- Program constants --------------------------- */

// Distance in cm at which an IR sensor is considered blocked when moving
// forward or backwards
const int LINEAR_CLEAR_THRESHOLD = -1;
// Distance in cm at which an IR sensor is considered blocked when turning
const int TURN_CLEAR_THRESHOLD = -1;

// Values below PWM_MIN will be treated as 0
const int PWM_MIN = 24; // one less than 10% power
// Values above PWM_MAX will be reduced to PWM_MAX
const int PWM_MAX = 50; // 20% power
// PWM value used when turning in place (+/- for each side, based on direction)
const int PWM_TURN = 75; // 30% power

// Time in milliseconds to continue the last command before stopping, in absence
// of a new command
const unsigned COMMAND_TIMEOUT = 200;
// Time in milliseconds between checking if the IR sensors are blocked
const unsigned IR_POLL = 200;
// Rate in ms at which encoder ticks are published
const unsigned ENC_POLL = 200;

// Min/max value of the tick count for under- and overflow
const unsigned ENC_BOUND = 32767;

/* ---------------------------- Arduino pin setup --------------------------- */

// Pins for connection to the left motor driver
const int L_ENA = 5;
const int L_ENB = 6;
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

// Pins for the encoders
const int FL_ENC = 18;
const int RL_ENC = 19;
const int FR_ENC = 20;
const int RR_ENC = 21;

// Model ID for the IR sensors, used internally in SharpIR
#define model SharpIR::GP2Y0A21YK0F

// Sets up the IR sensors, second parameter is the pin they're connected to
SharpIR IR_FL(model, A1); // Front left
SharpIR IR_FR(model, A2); // Front right
SharpIR IR_RR(model, A3); // Rear right
SharpIR IR_RL(model, A4); // Rear left
SharpIR IR_FC(model, A5); // Front center

/* --------------------- ROS publishers and message data -------------------- */

std_msgs::String msg;
ros::Publisher chatter("chatter", &msg);

std_msgs::Int16 FL_ticks;
ros::Publisher FL_Pub("FL_ticks", &FL_ticks);

std_msgs::Int16 FR_ticks;
ros::Publisher FR_Pub("FR_ticks", &FR_ticks);

std_msgs::Int16 RR_ticks;
ros::Publisher RR_Pub("RR_ticks", &RR_ticks);

std_msgs::Int16 RL_ticks;
ros::Publisher RL_Pub("RL_ticks", &RL_ticks);

/* ------------------------------ Program data ------------------------------ */

// These values are from -255 to 255, indicating required speed and direction
int pwmLeftReq = 0;
int pwmRightReq = 0;

// Tracks whether the wheels on each side have been told to move forward or
// backwards, which is only used for the encoders. Updated when PWM is set,
// remains unchanged when robot is ordered to stop.
bool left_moving_forward = true;
bool right_moving_forward = true;

// Timestamp in milliseconds of the last command
unsigned long lastCommandTime = 0;
// Timestamp in milliseconds of the last IR check
unsigned long lastIRTime = 0;
// Timestamp in milliseconds of the last publish of encoder data
unsigned long lastEncTime = 0; 

// IR sensor data
int fr_dist, fl_dist, rl_dist, rr_dist, fc_dist;

/* -------------------------------- Encoders -------------------------------- */

// Since the encoders only register ticks, not direction, these tick functions
// fudge the direction by looking at whether the robot was last told to move
// forward or backwards. This will usually work even for drift after the robot
// has been told to stop, but won't catch movement in a different direction than
// expected.

// When the tick count approaches an over- or underflow, it is re-centered to 0.
// This prevents continuous wrapping if, for example, the robot is rocking in
// place and the tick count is at an extreme. That would be problematic because
// the downstream velocity calculation ignores ticks from intervals when it
// wraps and would never yield a value.

void FL_tick() {
    if (abs(FL_ticks.data) == ENC_BOUND) {
        FL_ticks.data = 0;
    }
    
    if (left_moving_forward) {
        FL_ticks.data++;
    } else {
        FL_ticks.data--;
    }
}

void FR_tick() {
    if (abs(FR_ticks.data) == ENC_BOUND) {
        FR_ticks.data = 0;
    }

    if (left_moving_forward) {
        FR_ticks.data++;
    } else {
        FR_ticks.data--;
    }
}

void RR_tick() {
    if (abs(RR_ticks.data) == ENC_BOUND) {
        RR_ticks.data = 0;
    }

    if (right_moving_forward) {
        RR_ticks.data++;
    } else {
        RR_ticks.data--;
    }
}

void RL_tick() {
    if (abs(RL_ticks.data) == ENC_BOUND) {
        RL_ticks.data = 0;
    }

    if (right_moving_forward) {
        RL_ticks.data++;
    } else {
        RL_ticks.data--;
    }
}

/* ------------------------------- IR Sensors ------------------------------- */

void updateDistance() {
    fr_dist = IR_FR.getDistance();
    fl_dist = IR_FL.getDistance();
    rl_dist = IR_RL.getDistance();
    rr_dist = IR_RR.getDistance();
    fc_dist = IR_FC.getDistance();
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
            return false;
        } else if (pwmLeftReq < 0 && !backClear()) {
            // If moving backwards and rear is not clear
            return false;
        }

        // Else PWM is 0 and the robot isn't moving
        return true;
    } else if (!areaClear()) {
        // If turning and area is not clear
        return false;
    }

    return true;
}

/* ------------------------------ Motor Control ----------------------------- */

// Updates the required PWM values every time a new ROS command comes in, but
// does not update the values sent to the motors.
void calc_pwm(const geometry_msgs::Twist &cmdVel) {
    msg.data = "calculating pwm";
    chatter.publish(&msg);

    lastCommandTime = millis();

    // For now, this interprets incoming cmdVel messages with m/s and rad/s
    // values as percents of max PWM. Full forward is 1.00 m/s, reverse is -1.00
    // m/s, full left turn is 1.00 rad/s, full right turn is -1.00 rad/s, etc.
    // Uses linear.x and angular.z.

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
    // Constrain left
    if (abs(pwmLeftReq) < PWM_MIN) {
        pwmLeftReq = 0;
    } else if (abs(pwmLeftReq) > PWM_MAX) {
        // Sets the PWM to the max value, but with the same sign
        pwmLeftReq = PWM_MAX * ((pwmLeftReq > 0) - (pwmLeftReq < 0));
    }

    // Constrain right
    if (abs(pwmRightReq) < PWM_MIN) {
        pwmRightReq = 0;
    } else if (abs(pwmRightReq) > PWM_MAX) {
        // Sets the PWM to the max value, but with the same sign
        pwmRightReq = PWM_MAX * ((pwmRightReq > 0) - (pwmRightReq < 0));
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

        left_moving_forward = true;
    } else if (pwmLeftReq < 0) {
        // Left reverse
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 1);
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 1);

        left_moving_forward = false;
    } else {
        // Left stop
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 0);
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 0);

        // left_moving_forward remains at its last value to account for drift
    }

    if (pwmRightReq > 0) {
        // Right forward
        digitalWrite(R_INT1, 1);
        digitalWrite(R_INT2, 0);
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 1);

        right_moving_forward = true;
    } else if (pwmRightReq < 0) {
        // Right reverse
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 1);
        digitalWrite(R_INT3, 1);
        digitalWrite(R_INT4, 0);

        right_moving_forward = false;
    } else {
        // Right stop
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 0);
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 0);

        // right_moving_forward remains at its last value to account for drift
    }
    /* -------------------------------------------------------------------------- */

    // Writes the PWM values (which should already be constrained)
    analogWrite(L_ENA, abs(pwmLeftReq));
    analogWrite(L_ENB, abs(pwmLeftReq));
    analogWrite(R_ENA, abs(pwmRightReq));
    analogWrite(R_ENB, abs(pwmRightReq));
}

/* -------------------------------------------------------------------------- */

// Sets a ROS subscriber to handle velocity commands with the calc_pwm function
ros::Subscriber<geometry_msgs::Twist> CmdVel_Sub("cmd_vel", &calc_pwm);

// Runs once on Arduino startup
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

    // Attaches interrupts to encoder pins, calls given functions on rising edge
    attachInterrupt(digitalPinToInterrupt(FL_ENC), FL_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENC), FR_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RR_ENC), RR_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RL_ENC), RL_tick, RISING);

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
    nh.advertise(chatter);
    nh.advertise(FL_Pub);
    nh.advertise(FR_Pub);
    nh.advertise(RR_Pub);
    nh.advertise(RL_Pub);
    nh.subscribe(CmdVel_Sub);
}

// Main loop, runs repeatedly while Arduino is on
void loop() {
    // Handles incoming ROS messages (which calls calc_pwm per the CmdVel_Sub
    // subscriber)
    nh.spinOnce();

    unsigned long currentTime = millis();

    // Stops the robot if it has been more than COMMAND_TIMEOUT ms since the
    // last instruction
    if (currentTime - lastCommandTime > COMMAND_TIMEOUT) {
        msg.data = "timeout";
        chatter.publish(&msg);

        pwmLeftReq = 0;
        pwmRightReq = 0;
    } else { 
        // Checks the IR sensors if it has been at least IR_POLL ms since the
        // last check
        if (currentTime - lastIRTime > IR_POLL) {
            updateDistance();

            if (!checkClear()) {
                msg.data = "blocked";
                chatter.publish(&msg);

                pwmLeftReq = 0;
                pwmRightReq = 0;
            }
        }

        // Publishes encoder data if it has been at least ENC_POLL ms since the
        // last publish
        if (currentTime - lastEncTime > ENC_POLL) {
            lastEncTime = currentTime;

            FL_Pub.publish(&FL_ticks);
            FR_Pub.publish(&FR_ticks);
            RR_Pub.publish(&RR_ticks);
            RL_Pub.publish(&RL_ticks);
        }
    }

    set_pwm();
}