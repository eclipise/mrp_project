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

volatile unsigned short fl_forward_ticks[256];
volatile unsigned short fl_backward_ticks[256];
volatile unsigned short fr_forward_ticks[256];
volatile unsigned short fr_backward_ticks[256];
volatile unsigned short rl_forward_ticks[256];
volatile unsigned short rl_backward_ticks[256];
volatile unsigned short rr_forward_ticks[256];
volatile unsigned short rr_backward_ticks[256];

bool forward = true;
bool read_ticks = false;
bool do_test = true;

unsigned pwm = 0;

void fl_tick() {
    if (!read_ticks) {
        return;
    }

    if (forward) {
        fl_forward_ticks[pwm]++;
    } else {
        fl_backward_ticks[pwm]++;
    }
}

void fr_tick() {
    if (!read_ticks) {
        return;
    }

    if (forward) {
        fr_forward_ticks[pwm]++;
    } else {
        fr_backward_ticks[pwm]++;
    }
}

void rl_tick() {
    if (!read_ticks) {
        return;
    }

    if (forward) {
        rl_forward_ticks[pwm]++;
    } else {
        rl_backward_ticks[pwm]++;
    }
}

void rr_tick() {
    if (!read_ticks) {
        return;
    }

    if (forward) {
        rr_forward_ticks[pwm]++;
    } else {
        rr_backward_ticks[pwm]++;
    }
}

void print_ticks() {
    Serial.print("FL F: ");
    for (int i = 0; i < 256; i++) {
        Serial.print(fl_forward_ticks[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("FL B: ");
    for (int i = 0; i < 256; i++) {
        Serial.print(fl_backward_ticks[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("FR F: ");
    for (int i = 0; i < 256; i++) {
        Serial.print(fr_forward_ticks[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("FR B: ");
    for (int i = 0; i < 256; i++) {
        Serial.print(fr_backward_ticks[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("RL F: ");
    for (int i = 0; i < 256; i++) {
        Serial.print(rl_forward_ticks[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("RL B: ");
    for (int i = 0; i < 256; i++) {
        Serial.print(rl_backward_ticks[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("RR F: ");
    for (int i = 0; i < 256; i++) {
        Serial.print(rr_forward_ticks[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("RR B: ");
    for (int i = 0; i < 256; i++) {
        Serial.print(rr_backward_ticks[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void set_pwm(int pwm) {
    if (forward) {
        digitalWrite(L_INT1, 1);
        digitalWrite(L_INT2, 0);
        digitalWrite(L_INT3, 1);
        digitalWrite(L_INT4, 0);
        digitalWrite(R_INT1, 1);
        digitalWrite(R_INT2, 0);
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 1);
    } else {
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 1);
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 1);
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 1);
        digitalWrite(R_INT3, 1);
        digitalWrite(R_INT4, 0);
    }

    analogWrite(L_ENA, pwm);
    analogWrite(L_ENB, pwm);
    analogWrite(R_ENA, pwm);
    analogWrite(R_ENB, pwm);
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
    attachInterrupt(digitalPinToInterrupt(FL_ENC), fl_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENC), fr_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RL_ENC), rl_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RR_ENC), rr_tick, RISING);

    Serial.begin(57600);
}

void loop() {
    if (do_test) {
        for (int i = 0; i <= 255; i++) {
            pwm = i;

            Serial.println(pwm);

            // Initialization
            fl_forward_ticks[pwm] = 0;
            fr_forward_ticks[pwm] = 0;
            rl_forward_ticks[pwm] = 0;
            rr_forward_ticks[pwm] = 0;

            set_pwm(pwm);

            // Allows 2 seconds to get up to speed
            delay(2000);

            unsigned long time = millis();

            // Records 1 second worth of ticks
            read_ticks = true;
            while (millis() - time < 1000) {
            }
            read_ticks = false;

            set_pwm(0);
        }

        forward = false;

        for (int i = 0; i <= 255; i++) {
            pwm = i;

            Serial.println(pwm);

            // Initialization
            fl_backward_ticks[pwm] = 0;
            fr_backward_ticks[pwm] = 0;
            rl_backward_ticks[pwm] = 0;
            rr_backward_ticks[pwm] = 0;

            set_pwm(pwm);

            // Allows 2 seconds to get up to speed
            delay(2000);
            unsigned long time = millis();

            // Records 1 second worth of ticks
            read_ticks = true;
            while (millis() - time < 1000) {
            }
            read_ticks = false;

            set_pwm(0);
        }

        print_ticks();
        do_test = false;
    }
}