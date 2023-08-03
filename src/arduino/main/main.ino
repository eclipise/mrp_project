#include <SharpIR.h>

/* ----------------------------- Program config ----------------------------- */

const int BAUD_RATE = 57600;

// Distance in cm at which an IR sensor is considered blocked when moving
// forward or backwards
int LINEAR_CLEAR_THRESHOLD = 25;
// Distance in cm at which an IR sensor is considered blocked when turning
int TURN_CLEAR_THRESHOLD = 20;

// Values below PWM_MIN will be treated as 0
int PWM_MIN = 24; // 10% power
// Values above PWM_MAX will be reduced to PWM_MAX
int PWM_MAX = 50; // 20% power
// PWM value used when turning in place (+/- for each side, based on direction)
int PWM_TURN = 75; // 30% power

// Time in milliseconds to continue the last command before stopping, in absence
// of a new command
unsigned COMMAND_TIMEOUT = 200;
// Time in milliseconds between checking if the IR sensors are blocked. Should
// always be greater than 30 ms to prevent an internal delay in SharpIR.
const unsigned IR_POLL = 200;

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

// Sets up the IR sensors, second argument is the pin they're connected to
SharpIR IR_FL(model, A1); // Front left
SharpIR IR_FR(model, A2); // Front right
SharpIR IR_RR(model, A3); // Rear right
SharpIR IR_RL(model, A4); // Rear left
SharpIR IR_FC(model, A5); // Front center

/* ------------------------------ Program data ------------------------------ */

// Tracks whether the motors have been told to move forward or backwards, which
// is only used for the encoders. Updated when PWM is set, remains unchanged
// when robot is ordered to stop.
bool fl_moving_forward = true;
bool fr_moving_forward = true;
bool rr_moving_forward = true;
bool rl_moving_forward = true;

// Timestamp in milliseconds of the last command
unsigned long lastCmdTime = 0;
// Timestamp in milliseconds of the last IR check
unsigned long lastIRTime = 0;

// IR sensor data
int fl_dist, fr_dist, rr_dist, rl_dist, fc_dist;

volatile long fl_ticks, fr_ticks, rr_ticks, rl_ticks;

/* -------------------------------- Encoders -------------------------------- */

// Since the encoders only register ticks, not direction, these tick functions
// fudge the direction by looking at whether the robot was last told to move
// forward or backwards. This will usually work even for drift after the robot
// has been told to stop, but won't catch movement in a different direction than
// expected.

void FL_tick() {
    if (fl_moving_forward) {
        fl_ticks++;
    } else {
        fl_ticks--;
    }
}

void FR_tick() {
    if (fr_moving_forward) {
        fr_ticks++;
    } else {
        fr_ticks--;
    }
}

void RR_tick() {
    if (rr_moving_forward) {
        rr_ticks++;
    } else {
        rr_ticks--;
    }
}

void RL_tick() {
    if (rl_moving_forward) {
        rl_ticks++;
    } else {
        rl_ticks--;
    }
}

/* ------------------------------- IR Sensors ------------------------------- */

void updateIR() {
    fl_dist = IR_FL.getDistance();
    fr_dist = IR_FR.getDistance();
    rr_dist = IR_RR.getDistance();
    rl_dist = IR_RL.getDistance();
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
    if (fl_moving_forward == fr_moving_forward) {
        if (fl_moving_forward && !frontClear()) {
            // If moving forward and front is not clear
            return false;
        } else if (!fl_moving_forward && !backClear()) {
            // If moving backwards and rear is not clear
            return false;
        }
    } else if (!areaClear()) {
        // If turning and area is not clear
        return false;
    }

    return true;
}

/* --------------------------------- Motors --------------------------------- */

// int constrain_pwm(int pwm) {

//     // Constrain left
//     if (abs(pwmLeftReq) < PWM_MIN) {
//         pwmLeftReq = 0;
//     } else if (abs(pwmLeftReq) > PWM_MAX) {
//         // Sets the PWM to the max value, but with the same sign
//         pwmLeftReq = PWM_MAX * ((pwmLeftReq > 0) - (pwmLeftReq < 0));
//     }

//     // Constrain right
//     if (abs(pwmRightReq) < PWM_MIN) {
//         pwmRightReq = 0;
//     } else if (abs(pwmRightReq) > PWM_MAX) {
//         // Sets the PWM to the max value, but with the same sign
//         pwmRightReq = PWM_MAX * ((pwmRightReq > 0) - (pwmRightReq < 0));
//     }
// }

// TODO: check directions and PWM pins
void set_pwm(int fl_pwm, int fr_pwm, int rr_pwm, int rl_pwm) {
    // Front left
    if (fl_pwm > 0) {
        // Forward
        digitalWrite(L_INT1, 1);
        digitalWrite(L_INT2, 0);

        fl_moving_forward = true;
    } else if (fl_pwm < 0) {
        // Backward
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 1);

        fl_moving_forward = false;
    } else {
        // Stop
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 0);

        // fl_moving_forward remains at its last value to account for drift
    }

    analogWrite(L_ENA, fl_pwm);

    // Front right
    if (fr_pwm > 0) {
        // Forward
        digitalWrite(L_INT3, 1);
        digitalWrite(L_INT4, 0);

        fr_moving_forward = true;
    } else if (fr_pwm < 0) {
        // Backward
        digitalWrite(R_INT3, 1);
        digitalWrite(R_INT4, 0);

        fr_moving_forward = false;
    } else {
        // Stop
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 0);
    }

    analogWrite(L_ENB, fr_pwm);

    // Rear right
    if (rr_pwm > 0) {
        // Forward
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 1);

        rr_moving_forward = true;
    } else if (rr_pwm < 0) {
        // Backward
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 1);

        rr_moving_forward = false;
    } else {
        // Stop
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 0);
    }

    analogWrite(R_ENA, rr_pwm);

    // Rear left
    if (rl_pwm > 0) {
        // Forward
        digitalWrite(R_INT1, 1);
        digitalWrite(R_INT2, 0);

        rl_moving_forward = true;
    } else if (rl_pwm < 0) {
        // Backward
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 1);

        rl_moving_forward = false;
    } else {
        // Stop
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 0);
    }

    analogWrite(R_ENB, rl_pwm);
}

/* ----------------------------- Program control ---------------------------- */

void run_command(char cmd_sel, int arg1, int arg2, int arg3, int arg4) {
    switch (cmd_sel) {
    // Encoder read
    case 'e':
        Serial.print(fl_ticks);
        Serial.print(" ");
        Serial.print(fr_ticks);
        Serial.print(" ");
        Serial.print(rl_ticks);
        Serial.print(" ");
        Serial.println(rr_ticks);
        break;

    // Set motor pwm
    case 'm':
        lastCmdTime = millis();
        set_pwm(arg1, arg2, arg3, arg4);
        break;

    // IR read
    case 'i':
        updateIR();

        Serial.print(fl_dist);
        Serial.print(" ");
        Serial.print(fc_dist);
        Serial.print(" ");
        Serial.print(fr_dist);
        Serial.print(" ");
        Serial.print(rl_dist);
        Serial.print(" ");
        Serial.println(rr_dist);
        break;

    // IR check
    case 'c':
        updateIR();
        Serial.println(checkClear());
        break;

    // IR threshold config
    case 't':
        LINEAR_CLEAR_THRESHOLD = arg1;
        TURN_CLEAR_THRESHOLD = arg2;
        break;

    // Command timeout config
    case 'o':
        COMMAND_TIMEOUT = arg1;
        break;

    // Max speed config
    case 's': 
        PWM_MIN = arg1;
        PWM_MAX = arg2;
        PWM_TURN = arg3;
        break;

    default:
        Serial.println("Invalid command.");
        break;
    }
}

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

    // Sets the encoder pins to input mode
    pinMode(FL_ENC, INPUT);
    pinMode(FR_ENC, INPUT);
    pinMode(RR_ENC, INPUT);
    pinMode(RL_ENC, INPUT);

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

    Serial.begin(BAUD_RATE);
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');

        char cmd_sel = '\0';
        int arg1 = 0, arg2 = 0, arg3 = 0, arg4 = 0;

        int num_parsed = sscanf(command.c_str(), "%c %d %d %d %d", &cmd_sel, &arg1, &arg2, &arg3, &arg4);

        if (num_parsed > 0) {
            run_command(cmd_sel, arg1, arg2, arg3, arg4);
        }
    }

    unsigned long currentTime = millis();

    if (currentTime - lastCmdTime > COMMAND_TIMEOUT) {
        // Stops robot if COMMAND_TIMEOUT ms have elapsed since the last command
        set_pwm(0, 0, 0, 0);
    } else if (currentTime - lastIRTime > IR_POLL) {
        // Checks the IR sensors if it has been at least IR_POLL ms since the last check
        lastIRTime = currentTime;

        updateIR();

        if (!checkClear()) {
            set_pwm(0, 0, 0, 0);
        }
    }
}