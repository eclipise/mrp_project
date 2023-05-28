#include <SharpIR.h>

int motorPinsLeft[6] = {5, 44, 46, 48, 50, 4};  // Pins for left motor
int motorPinsRight[6] = {3, 45, 47, 49, 51, 2}; // Pins for right motor

int maxi = 255; // Max bits, representing x00ff
int leftSpeed, rightSpeed;
// int theTorque = 1.0625; // Conversion of how many bits will result in 1rpm

int speedInput;
int turnInput;
int timeInput;
bool params_set = false;

#define IR1 A1
#define IR2 A2
#define model 1080

SharpIR SharpIR1(IR1, model); // Setting up IR sensor 1
SharpIR SharpIR2(IR2, model); // Setting up IR sensor 2

int dis[2] = {0, 0}; // Building a matrix for placing of sensor values

void setup() {
    // put your setup code here, to run once:  Setting everything as output
    for (int i = 0; i < 6; i++) {
        pinMode(motorPinsLeft[i], OUTPUT);
        pinMode(motorPinsRight[i], OUTPUT);
    }

    Serial.begin(9600); // Baud rate selected
}

void getDistance(int *dis) {

    unsigned long pepe1 = millis(); // takes the time before the loop on the library begins

    int dis1 = SharpIR1.distance(); // this returns the distance to the object you're measuring
    int dis2 = SharpIR2.distance();

    dis[0] = dis1; // Writing value of IR sensor onto location 0 of matrix
    dis[1] = dis2; // Writing value of IR sensor onto location 1 of matrix
    // Serial.println(dis1);
    // Serial.println(dis2);
}

// speed and turn should be [-1.0, 1.0]
void moveRobot(float theSpeed, float theTurn, int t) {

    getDistance(dis);

    if (dis[0] < 20 || dis[1] < 30) {
        Serial.println("STOP");
        digitalWrite(motorPinsLeft[1], 0);  // Pin 45, int1
        digitalWrite(motorPinsLeft[2], 0);  // Pin 47, int2
        digitalWrite(motorPinsLeft[3], 0);  // Pin 49, int4
        digitalWrite(motorPinsLeft[4], 0);  // Pin 51, int3
        digitalWrite(motorPinsRight[1], 0); // Pin 44, int1
        digitalWrite(motorPinsRight[2], 0); // Pin 46, int2
        digitalWrite(motorPinsRight[3], 0); // Pin 48, int4
        digitalWrite(motorPinsRight[4], 0); // Pin 50, int3
        return;
    }

    // Desired speed, turn and duration t
    theSpeed *= maxi; // The per unit speed provided times the maximum bits, to see how many bits it corresponds to
    theTurn *= maxi;  // The per unit turn provided times the maximum bits, to see how many bits it corresponds to
    // theTorque *= theSpeed; // Math conversion of the bits to rpm for the motors

    // Individual Speeds
    leftSpeed = int(theSpeed - theTurn);  // If my turn is positive it will turn to the left by making left motors slower and right motors faster
    rightSpeed = int(theSpeed + theTurn); // If my turn is negative it will turn to the right by making left motors faster and right motors slower

    // Limit
    leftSpeed = constrain(leftSpeed, -maxi, maxi);   // Constrain the values to not surpass -255 to 255. As that is 100%
    rightSpeed = constrain(rightSpeed, -maxi, maxi); // In case if someone enters more than 1.0 to mySpeed or myTurn

    // Send Speed
    analogWrite(motorPinsLeft[0], abs(leftSpeed));   // Pin 3
    analogWrite(motorPinsLeft[5], abs(leftSpeed));   // Pin 2
    analogWrite(motorPinsRight[0], abs(rightSpeed)); // Pin 5
    analogWrite(motorPinsRight[5], abs(rightSpeed)); // Pin 4
    // char buffer[40];
    // sprintf(buffer, "Speed for the left motors is %d ", leftSpeed");
    // Serial.printls(buffer);

    Serial.println("Speed for the left motors");
    Serial.println(leftSpeed);
    Serial.println("Speed for the right motors");
    Serial.println(rightSpeed);
    // Serial.println("Speed for the left motors");
    // Serial.println(theTorque);

    if (leftSpeed > 0) {                   // Forward Left
        digitalWrite(motorPinsLeft[1], 1); // Pin 45, int1
        digitalWrite(motorPinsLeft[2], 0); // Pin 47, int2
        digitalWrite(motorPinsLeft[3], 1); // Pin 49, int4
        digitalWrite(motorPinsLeft[4], 0); // Pin 51, int3
        Serial.println("------------Forward Left");
    } else { // Backward Left
        digitalWrite(motorPinsLeft[1], 0);
        digitalWrite(motorPinsLeft[2], 1);
        digitalWrite(motorPinsLeft[3], 0);
        digitalWrite(motorPinsLeft[4], 1);
        Serial.println("------------Backward Left");
    }

    if (rightSpeed > 0) {                   // Forward Right
        digitalWrite(motorPinsRight[1], 1); // Pin 44, int1
        digitalWrite(motorPinsRight[2], 0); // Pin 46, int2
        digitalWrite(motorPinsRight[3], 0); // Pin 48, int4
        digitalWrite(motorPinsRight[4], 1); // Pin 50, int3
        Serial.println("------------Forward Right");
    } else { // Backwards Right
        digitalWrite(motorPinsRight[1], 0);
        digitalWrite(motorPinsRight[2], 1);
        digitalWrite(motorPinsRight[3], 1);
        digitalWrite(motorPinsRight[4], 0);
        Serial.println("------------Backward Right");
    }

    delay(t);
    Serial.println("STOP");
    digitalWrite(motorPinsLeft[1], 0);  // Pin 45, int1
    digitalWrite(motorPinsLeft[2], 0);  // Pin 47, int2
    digitalWrite(motorPinsLeft[3], 0);  // Pin 49, int4
    digitalWrite(motorPinsLeft[4], 0);  // Pin 51, int3
    digitalWrite(motorPinsRight[1], 0); // Pin 44, int1
    digitalWrite(motorPinsRight[2], 0); // Pin 46, int2
    digitalWrite(motorPinsRight[3], 0); // Pin 48, int4
    digitalWrite(motorPinsRight[4], 0); // Pin 50, int3
}

void loop() {
    // put your main code here, to run repeatedly:

    if (Serial.available() > 0) {
        // read the incoming message
        String message = Serial.readStringUntil('\n');
        Serial.print("Received message: ");
        Serial.println(message);

        // check message length
        if (message.length() < 7) {
            Serial.println("Invalid message length");
            return;
        }

        // parse the parameters from the message
        char buffer[30];
        message.toCharArray(buffer, 30);
        int result = sscanf(buffer, "%d %d %d", &speedInput, &turnInput, &timeInput);
        Serial.print("sscanf result: ");
        Serial.println(result);

        // check for parsing errors
        if (result != 3) {
            Serial.println("Error parsing parameters");
            return;
        }

        // print the parameters to the serial monitor
        Serial.print("Received parameters: ");
        Serial.print(speedInput);
        Serial.print(" ");
        Serial.print(turnInput);
        Serial.print(" ");
        Serial.println(timeInput);

        moveRobot(speedInput, turnInput, timeInput);
    }
}