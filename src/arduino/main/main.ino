#include <PID_v1.h>
#include <SharpIR.h>
#include <util/atomic.h>

/* ----------------------------- Program config ----------------------------- */

#define BAUD_RATE 57600

// Distance in cm at which an IR sensor is considered blocked when moving
// forward or backwards
int LINEAR_CLEAR_THRESHOLD = 25;
// Distance in cm at which an IR sensor is considered blocked when turning
int TURN_CLEAR_THRESHOLD = 20;

// Values below PWM_MIN will be treated as 0
const int PWM_MIN = 24; // 10% power
// Values above PWM_MAX will be reduced to PWM_MAX
const int PWM_MAX = 50; // 20% power

// Time in milliseconds to continue the last command before stopping, in absence
// of a new command.
unsigned COMMAND_TIMEOUT = 200;
// Should always be greater than 30 ms to prevent an internal delay in SharpIR.
const unsigned IR_POLL = 200;

const unsigned PID_RATE = 20;

// ACS712 ammeter constants
const float AMMETER_VOLTAGE = 5.0;
const float AMMETER_SENSITIVITY = 0.066;
const int AMMETER_MAX = 1024;   // Ammeter raw value is in range [0, 1024]
const float MIN_CURRENT = 0.25; // Values below this are treated as 0 amps

const int ENC_COUNTS_PER_REV = 40;
const double WHEEL_RADIUS = 0.0762; // meters

// Default PID config
const int Kp = 2;
const int Ki = 5;
const int Kd = 1;

/* ---------------------------- Arduino pin setup --------------------------- */

// Pins for connection to the left motor driver
const int L_ENA = 5; // fl
const int L_ENB = 6; // rl
const int L_INT1 = 44;
const int L_INT2 = 46;
const int L_INT3 = 48;
const int L_INT4 = 50;

// Pins for connection to the right motor driver
const int R_ENA = 2; // fr
const int R_ENB = 3; // rr
const int R_INT1 = 45;
const int R_INT2 = 47;
const int R_INT3 = 49;
const int R_INT4 = 51;

// Pins for the encoders
const int FL_ENC = 18;
const int FR_ENC = 20;
const int RL_ENC = 19;
const int RR_ENC = 21;

// Model ID for the IR sensors, used internally in SharpIR
#define model SharpIR::GP2Y0A21YK0F

// Sets up the IR sensors, second argument is the pin they're connected to
SharpIR IR_FL(model, A1);
SharpIR IR_FC(model, A5);
SharpIR IR_FR(model, A2);
SharpIR IR_RL(model, A4);
SharpIR IR_RR(model, A3);

// Pin for the ammeter
#define AMMETER_PIN A0

/* ------------------------------ Program data ------------------------------ */

// Tracks whether the motors have been told to move forward or backwards, which
// is only used for the encoders. Updated when PWM is set, remains unchanged
// when robot is ordered to stop.
bool fl_moving_forward = true;
bool fr_moving_forward = true;
bool rl_moving_forward = true;
bool rr_moving_forward = true;

// Timestamp in milliseconds of the last command
unsigned long lastCmdTime = 0;
// Timestamp in milliseconds of the last IR check
unsigned long lastIRTime = 0;

// IR sensor data
int fl_dist, fc_dist, fr_dist, rl_dist, rr_dist;

// Any access to these variables must be wrapped in an ATOMIC_BLOCK to prevent
// interrupts mid-read, since they are larger than 1 byte.
volatile long fl_ticks, fr_ticks, rl_ticks, rr_ticks;

// Tick value at last velocity calculation
long prev_fl_ticks = 0, prev_fr_ticks = 0, prev_rl_ticks = 0, prev_rr_ticks = 0;

// Time of last velocity calculation in microseconds
unsigned long prev_fl_time = 0, prev_fr_time = 0, prev_rl_time = 0, prev_rr_time = 0;

// Current velocity in rad/s
double fl_vel = 0, fr_vel = 0, rl_vel = 0, rr_vel = 0;

// Desired velocity in rad/s
double target_fl_vel = 0, target_fr_vel = 0, target_rl_vel = 0, target_rr_vel = 0;

// Current PWM value sent to the motors (double because of PID)
double fl_pwm = 0, fr_pwm = 0, rl_pwm = 0, rr_pwm = 0;

/* ----------------------------------- PID ---------------------------------- */

// These default to off
PID fl_PID(&fl_vel, &fl_pwm, &target_fl_vel, Kp, Ki, Kd, DIRECT);
PID fr_PID(&fr_vel, &fr_pwm, &target_fr_vel, Kp, Ki, Kd, DIRECT);
PID rl_PID(&rl_vel, &rl_pwm, &target_rl_vel, Kp, Ki, Kd, DIRECT);
PID rr_PID(&rr_vel, &rr_pwm, &target_rr_vel, Kp, Ki, Kd, DIRECT);

void enable_PID() {
    fl_PID.SetMode(1);
    fr_PID.SetMode(1);
    rl_PID.SetMode(1);
    rr_PID.SetMode(1);
}

void disable_PID() {
    fl_PID.SetMode(0);
    fr_PID.SetMode(0);
    rl_PID.SetMode(0);
    rr_PID.SetMode(0);
}

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

void RL_tick() {
    if (rl_moving_forward) {
        rl_ticks++;
    } else {
        rl_ticks--;
    }
}

void RR_tick() {
    if (rr_moving_forward) {
        rr_ticks++;
    } else {
        rr_ticks--;
    }
}

void calc_vel(volatile long &ticks, long &prev_ticks, unsigned long &prev_time, double &vel) {
    int local_ticks = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        local_ticks = ticks;
    }

    long current_time = micros();
    double deltaT = double(current_time - prev_time) / 1.0e6; // Interval in seconds
    double velocity = (prev_ticks - local_ticks) / deltaT;
    prev_ticks = local_ticks;
    prev_time = current_time;

    vel = (velocity / ENC_COUNTS_PER_REV) * 2 * PI; // Converts to rad/s
}

/* ------------------------------- IR Sensors ------------------------------- */

void updateIR() {
    fl_dist = IR_FL.getDistance();
    fc_dist = IR_FC.getDistance();
    fr_dist = IR_FR.getDistance();
    rl_dist = IR_RL.getDistance();
    rr_dist = IR_RR.getDistance();
}

bool frontClear() {
    return fl_dist > LINEAR_CLEAR_THRESHOLD &&
           fc_dist > LINEAR_CLEAR_THRESHOLD &&
           fr_dist > LINEAR_CLEAR_THRESHOLD;
}

bool backClear() {
    return rl_dist > LINEAR_CLEAR_THRESHOLD &&
           rr_dist > LINEAR_CLEAR_THRESHOLD;
}

bool areaClear() {
    return fl_dist > TURN_CLEAR_THRESHOLD &&
           fc_dist > TURN_CLEAR_THRESHOLD &&
           fr_dist > TURN_CLEAR_THRESHOLD &&
           rl_dist > TURN_CLEAR_THRESHOLD &&
           rr_dist > TURN_CLEAR_THRESHOLD;
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

void set_pwm() {
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

    analogWrite(L_ENA, abs(fl_pwm));

    // Front right
    if (fr_pwm > 0) {
        // Forward
        digitalWrite(R_INT1, 1);
        digitalWrite(R_INT2, 0);

        fr_moving_forward = true;
    } else if (fr_pwm < 0) {
        // Backward
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 1);

        fr_moving_forward = false;
    } else {
        // Stop
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 0);
    }

    analogWrite(R_ENA, abs(fr_pwm));

    // Rear left
    if (rl_pwm > 0) {
        // Forward
        digitalWrite(L_INT3, 1);
        digitalWrite(L_INT4, 0);

        rl_moving_forward = true;
    } else if (rl_pwm < 0) {
        // Backward
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 1);

        rl_moving_forward = false;
    } else {
        // Stop
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 0);
    }

    analogWrite(L_ENB, abs(rl_pwm));

    // Rear right
    if (rr_pwm > 0) {
        // Forward
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 1);

        rr_moving_forward = true;
    } else if (rr_pwm < 0) {
        // Backward
        digitalWrite(R_INT3, 1);
        digitalWrite(R_INT4, 0);

        rr_moving_forward = false;
    } else {
        // Stop
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 0);
    }

    analogWrite(R_ENB, abs(rr_pwm));
}

/* --------------------------------- Ammeter -------------------------------- */

float readAmmeter() {
    int raw_data = analogRead(AMMETER_PIN);
    float voltage = (raw_data / float(AMMETER_MAX)) * AMMETER_VOLTAGE;
    float current = (voltage - AMMETER_VOLTAGE / 2) / AMMETER_SENSITIVITY;

    if (abs(current) < MIN_CURRENT) {
        current = 0;
    }

    return current;
}

/* ----------------------------- Program control ---------------------------- */

void run_command(char cmd_sel, int arg1, int arg2, int arg3, int arg4) {
    switch (cmd_sel) {
    // Set motor PWM (open loop)
    case 'o':
        lastCmdTime = millis();

        disable_PID();

        fl_pwm = arg1;
        fr_pwm = arg2;
        rl_pwm = arg3;
        rr_pwm = arg4;

        Serial.println("OK");
        break;

    // Set motor velocity in rad/s (closed loop)
    case 'm':
        lastCmdTime = millis();

        target_fl_vel = arg1;
        target_fr_vel = arg2;
        target_rl_vel = arg3;
        target_rr_vel = arg4;

        enable_PID();

        Serial.println("OK");
        break;

    // Max PWM config
    case 's':
        fl_PID.SetOutputLimits(arg1, arg2);
        fr_PID.SetOutputLimits(arg1, arg2);
        rl_PID.SetOutputLimits(arg1, arg2);
        rr_PID.SetOutputLimits(arg1, arg2);

        Serial.println("OK");
        break;

    // PID config
    case 'p':
        fl_PID.SetTunings(arg1, arg2, arg3);
        fr_PID.SetTunings(arg1, arg2, arg3);
        rl_PID.SetTunings(arg1, arg2, arg3);
        rr_PID.SetTunings(arg1, arg2, arg3);

        Serial.println("OK");
        break;

    // Command timeout config
    case 't':
        COMMAND_TIMEOUT = arg1;
        Serial.println("OK");
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

    // IR distance config
    case 'd':
        LINEAR_CLEAR_THRESHOLD = arg1;
        TURN_CLEAR_THRESHOLD = arg2;
        Serial.println("OK");
        break;

    // Read encoders
    case 'e':
        int local_fl_ticks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            local_fl_ticks = fl_ticks;
        }

        int local_fr_ticks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            local_fr_ticks = fr_ticks;
        }

        int local_rl_ticks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            local_rl_ticks = rl_ticks;
        }

        int local_rr_ticks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            local_rr_ticks = rr_ticks;
        }

        Serial.print(local_fl_ticks);
        Serial.print(" ");
        Serial.print(local_fr_ticks);
        Serial.print(" ");
        Serial.print(local_rl_ticks);
        Serial.print(" ");
        Serial.println(local_rr_ticks);
        break;

    // Reset encoders
    case 'r':
        fl_ticks = 0;
        fr_ticks = 0;
        rl_ticks = 0;
        rr_ticks = 0;
        Serial.println("OK");
        break;

    // Read velocity
    case 'v':
        Serial.print(fl_vel);
        Serial.print(" ");
        Serial.print(fr_vel);
        Serial.print(" ");
        Serial.print(rl_vel);
        Serial.print(" ");
        Serial.println(rr_vel);
        break;

    // Read ammeter
    case 'a':
        Serial.println(readAmmeter());
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
    pinMode(RL_ENC, INPUT);
    pinMode(RR_ENC, INPUT);

    // Attaches interrupts to encoder pins, calls given functions on rising edge
    attachInterrupt(digitalPinToInterrupt(FL_ENC), FL_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENC), FR_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RL_ENC), RL_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RR_ENC), RR_tick, RISING);

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

    // PID config
    fl_PID.SetSampleTime(PID_RATE);
    fr_PID.SetSampleTime(PID_RATE);
    rl_PID.SetSampleTime(PID_RATE);
    rr_PID.SetSampleTime(PID_RATE);

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
        disable_PID();

        fl_pwm = 0;
        fr_pwm = 0;
        rl_pwm = 0;
        rr_pwm = 0;
    } else if (currentTime - lastIRTime > IR_POLL) {
        // Checks the IR sensors if it has been at least IR_POLL ms since the last check
        lastIRTime = currentTime;

        updateIR();

        if (!checkClear()) {
            disable_PID();

            fl_pwm = 0;
            fr_pwm = 0;
            rl_pwm = 0;
            rr_pwm = 0;
        }
    }

    calc_vel(fl_ticks, prev_fl_ticks, prev_fl_time, fl_vel);
    calc_vel(fr_ticks, prev_fr_ticks, prev_fr_time, fr_vel);
    calc_vel(rl_ticks, prev_rl_ticks, prev_rl_time, rl_vel);
    calc_vel(rr_ticks, prev_rr_ticks, prev_rr_time, rr_vel);

    // Updates the PWM values if PID is enabled
    fl_PID.Compute();
    fr_PID.Compute();
    rl_PID.Compute();
    rr_PID.Compute();

    set_pwm();
}