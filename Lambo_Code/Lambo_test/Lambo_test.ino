/*
  ******************************************************************************
  * File Name     : Lambo_test.ino
  * Discription   : This is the TURNING line following algorithm for the robot, Lambo Rambo. 
  * Contributors  : Cole Stortz (main), Duncan Wood (second), Yannis Cassis, Kian Roseborough, Ala Assaf
  * Start Date    : 4/8/2024 
  * Last Modified : 5/20/2024

  * Comments:
  * - Parts; 3 motors, 4 color sensors, two 3d printed hexagons, teensy 4.1 microcontroller
  * - Will follow a black line and if it reaches the goal, go the opposite direction
  ******************************************************************************
  * @attention
  *
  * Copyright (c) LAMBO TEAM
  *
  ******************************************************************************
*/

// Pin connections for Motor A
#define MOTOR1_PWM 6
#define MOTOR1_CWISE 7
#define MOTOR1_ANTI_CWISE 8

// Pin connections for Motor B
#define MOTOR2_PWM 0
#define MOTOR2_CWISE 1
#define MOTOR2_ANTI_CWISE 2

// Pin connections for Motor C
#define MOTOR3_PWM 3
#define MOTOR3_CWISE 4
#define MOTOR3_ANTI_CWISE 5

// Motor Speed (PWM) value out of 255
#define PWM_VAL 200

// TCS3200 color sensor pin assignments
// Define the digital pins the sensors are connected to on the Teensy
const int numSensors = 4; // Number of color sensors
const int S0[numSensors] = {33, 31, 14, 19}; // S0 pins for each sensor
const int S1[numSensors] = {34, 32, 15, 20}; // S1 pins for each sensor
const int S2[numSensors] = {35, 40, 16, 21}; // S2 pins for each sensor
const int S3[numSensors] = {36, 41, 17, 22}; // S3 pins for each sensor
const int sensorOut[numSensors] = {37, 13, 18, 23}; // Sensor output pins for each sensor

// Calibrate the color sensors using white and black paper. Take the values reported when white paper is plac\ed under the sensor to be the minimum and values reported when black paper is placed under to be the maximum
int redMin[numSensors] = {39, 16, 41, 62};
int redMax[numSensors] = {387, 207, 331, 510};
int greenMin[numSensors] = {40, 13, 42, 49};
int greenMax[numSensors] = {453, 167, 343, 501};
int blueMin[numSensors] = {39, 17, 46, 66};
int blueMax[numSensors] = {417, 213, 372, 451};

// Define the analog input pins for the joystick axes
#define JOYSTICK_X_PIN A10 // Pin number used for xValue
#define JOYSTICK_Y_PIN A11 // Pin number used for yValue

// Define threshold values to consider for joystick dead zone
const int joyThreshold = 10;


#define ECHO_PIN 12 // attach Echo of HC-SR04
#define TRIG_PIN 11 //attach Trig of HC-SR04
long duration;
int distance;


// Function prototypes
void setup();
void loop();
void stopMotors();
void moveMotorsForward();
void moveMotorsBackward();
void moveMotorsRight();
void moveMotorsLeft();
void moveMotorsForwardRight();
void moveMotorsForwardLeft();
void moveMotorsBackwardRight();
void moveMotorsBackwardLeft();
void moveMotorsTurnLeft(); 
void moveMotorsTurnRight();
void moveTest();
void ultrasonicRead();
unsigned long readColorFrequency(int sensorIndex, int s2, int s3);
int mapValue(int x, int in_min, int in_max, int out_min, int out_max);

void setup() {
  Serial.begin(57600); // initialize serial connection for debugging

  // Set all motor controller pins as outputs
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR1_CWISE, OUTPUT);
  pinMode(MOTOR1_ANTI_CWISE, OUTPUT);
  pinMode(MOTOR2_CWISE, OUTPUT);
  pinMode(MOTOR2_ANTI_CWISE, OUTPUT);
  pinMode(MOTOR3_CWISE, OUTPUT);
  pinMode(MOTOR3_ANTI_CWISE, OUTPUT);

  // Set the joystick pins as input
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // Initial state - Turn off all the motors
  stopMotors();

  // Set up pins for each color sensor
  for (int i = 0; i < numSensors; i++) {
    pinMode(S0[i], OUTPUT);
    pinMode(S1[i], OUTPUT);
    pinMode(S2[i], OUTPUT);
    pinMode(S3[i], OUTPUT);
    pinMode(sensorOut[i], INPUT);
    
    // Set S0 and S1 for frequency scaling to 20%
    digitalWrite(S0[i], HIGH); // S0=1
    digitalWrite(S1[i], LOW);  // S1=0, for 20% frequency scaling
  }

}

void loop() {
  //unsigned long xPosition = analogRead(JOYSTICK_X_PIN);
  //unsigned long yPosition = analogRead(JOYSTICK_Y_PIN);

  // Set motor speeds
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL);

  // // Color sensor reading and action
  int colorsensorValues[numSensors] = {0};
  
  Serial.println(' ');
  Serial.println(' ');
  Serial.println(' ');
  Serial.println(' ');
  Serial.println(' ');
  Serial.println(' ');
  Serial.println(' ');
  for (int i = 0; i < numSensors; i++) {
    unsigned long redFrequency, greenFrequency, blueFrequency;

    // Read frequencies for each sensor
    redFrequency = readColorFrequency(i, LOW, LOW);
    blueFrequency = readColorFrequency(i, LOW, HIGH);
    greenFrequency = readColorFrequency(i, HIGH, HIGH);

    // Map the raw frequencies to the calibrated range
    int mappedRed = mapValue(redFrequency, redMin[i], redMax[i], 0, 255);
    int mappedGreen = mapValue(greenFrequency, greenMin[i], greenMax[i], 0, 255);
    int mappedBlue = mapValue(blueFrequency, blueMin[i], blueMax[i], 0, 255);

    // Debuging: print the unmapped values
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" - Red: ");
    Serial.print(redFrequency);
    Serial.print(" | Green: ");
    Serial.print(blueFrequency);
    Serial.print(" | Blue: ");
    Serial.println(greenFrequency);

    // Debugging: print the mapped values
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" - Red: ");
    Serial.print(mappedRed);
    Serial.print(" | Green: ");
    Serial.print(mappedGreen);
    Serial.print(" | Blue: ");
    Serial.println(mappedBlue);

    // Determine the color based on the mapped values
    if (mappedRed < 65 && mappedGreen < 65 && mappedBlue < 65) {
      colorsensorValues[i] = 1;
      Serial.println("Color Est: Yellow");
    } else if (mappedRed > 100 && mappedGreen > 160 && mappedBlue > 100){
      colorsensorValues[i] = 5;
      Serial.println("Color Est: Black");
    } else if (mappedRed < mappedBlue && mappedRed < mappedGreen) {
      colorsensorValues[i] = 2;
      Serial.println("Color Est: Red");
    } else if (mappedBlue < mappedRed && mappedBlue < mappedGreen) {
      colorsensorValues[i] = 3;
      Serial.println("Color Est: Blue");
    } else if (mappedGreen < mappedRed && mappedGreen) {
      colorsensorValues[i] = 4;
      Serial.println("Color Est: Green");
    } else {
      Serial.println("Color not recognized");
    }
  }
  ultrasonicRead();
  delay(200);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////MOTOR DEFINITION///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


void stopMotors() {
  // Stop all motors
  analogWrite(MOTOR2_PWM, PWM_VAL);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsForward() {
  Serial.println("Forward");
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotorsBackward() {
  Serial.println("Backward");
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, HIGH);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsRight() {
  Serial.println("Right");
  int adjustedSpeed = PWM_VAL * 0.5; // Decrease the speed
  analogWrite(MOTOR2_PWM, adjustedSpeed * 2); // Double the speed of motor 2 and adjust the other motors to the adjustedSpeed
  analogWrite(MOTOR1_PWM, adjustedSpeed);
  analogWrite(MOTOR3_PWM, adjustedSpeed);
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, HIGH);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsLeft() {
  Serial.println("Left");
  int adjustedSpeed = PWM_VAL * 0.5; // Decrease the speed
  analogWrite(MOTOR2_PWM, adjustedSpeed * 2); // Double the speed of motor 2 and adjust the other motors to the adjustedSpeed
  analogWrite(MOTOR1_PWM, adjustedSpeed);
  analogWrite(MOTOR3_PWM, adjustedSpeed);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotorsForwardRight() {
  Serial.println("Forward Right");
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsForwardLeft() {
  Serial.println("Forward Left");
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotorsBackwardRight() {
  Serial.println("Backward Right");
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsBackwardLeft() {
  Serial.println("Backward Left");
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, HIGH);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

// Other motor direction functions remain unchanged

// Additional motor direction functions
void moveMotorsTurnLeft() {
  Serial.println("Turn Left");
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotorsTurnRight() {
  Serial.println("Turn Right");
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, HIGH);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}
void moveMotorsTurnLeftreverse() {
  Serial.println("Turn Left");
  int adjustedSpeed = PWM_VAL; // Decrease the speed
  analogWrite(MOTOR2_PWM, adjustedSpeed); // Double the speed of motor 2 and adjust the other motors to the adjustedSpeed
  analogWrite(MOTOR1_PWM, adjustedSpeed * 1.25);
  analogWrite(MOTOR3_PWM, adjustedSpeed);
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, HIGH);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsTurnRightreverse() {
  Serial.println("Turn Right");
  int adjustedSpeed = PWM_VAL; // Decrease the speed
  analogWrite(MOTOR2_PWM, adjustedSpeed); // Double the speed of motor 2 and adjust the other motors to the adjustedSpeed
  analogWrite(MOTOR1_PWM, adjustedSpeed);
  analogWrite(MOTOR3_PWM, adjustedSpeed * 1.25);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotors180() {
  Serial.println("spinning");
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
  delay(900);
  moveMotorsForward();
}

// Ultrasonic sensor function remains unchanged

void moveTest() {
  delay(1000);
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL);
  moveMotorsForward();
  delay(200);
  moveMotorsBackward();
  delay(400);
  moveMotorsForward();
  delay(200);
  moveMotorsLeft();
  delay(200);
  moveMotorsRight();
  delay(400);
  moveMotorsLeft();
  delay(200);
  stopMotors();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// END ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////ULTRASONIC DEFINITION//////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void ultrasonicRead() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(10);

  // Reads the ECHO_PIN, returns the sound wave in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (send & receive)

  Serial.print(distance);
  Serial.println("cm");
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// END ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////COLOR SENSOR DEFINITION//////////////////////////////
////////////////////////////////////////////////////////////////////////////////

unsigned long readColorFrequency(int sensorIndex, int s2, int s3) {
  // Set S pins for the specific sensor
  digitalWrite(S2[sensorIndex], s2);
  digitalWrite(S3[sensorIndex], s3);
  // Read the output frequency from the color sensor using pulseIn with a timeout
  return pulseIn(sensorOut[sensorIndex], LOW, 1000000);
}

int mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// END ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
