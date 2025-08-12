/*
  ******************************************************************************
  * File Name     : Lambo_LineFollow.ino
  * Discription   : This is the TURNING line following algorithm for the robot, Lambo Rambo. 
  * Contributors  : Cole Stortz (main), Duncan Wood (second), Yannis Cassis, Kian Roseborough, Ala Assaf
  * Start Date    : 4/8/2024 
  * Last Modified : 5/20/2024

  * Comments:
  * - Parts; 3 motors, Two color sensors, two 3d printed hexagons, teensy 4.1 microcontroller
  * - Will follow a black line and if it reaches a blue line it will flip 180
  ******************************************************************************
  * @attention
  *
  * Copyright (c) LAMBO TEAM
  *
  ******************************************************************************
*/
#include <Servo.h>

Servo Arm;
Servo Gripper;

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
#define PWM_VAL 100
#define PWM_VAL_MOTOR3 100
#define PWM_VAL_MOTOR2 100

// Define the colors
#define BLACK 5
#define WHITE 1
#define YELLOW 6
#define GREEN 4
#define BLUE 3
#define RED 2

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

// Ultrasonic sensor pins and variables
#define ECHO_PIN 12 // attach Echo of HC-SR04
#define TRIG_PIN 11 //attach Trig of HC-SR04
long duration;
int distance;

// Variabes for the Algorithm
int x_pos = 0;
int x_floating = 35; //35 is start on the left, change to 0 to start on the right
int delayCondition = 0;
int move_val = 0;
int y_pos = 0;
int colorsensorValues[numSensors] = {0};

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
void readColorSensors();
unsigned long readColorFrequency(int sensorIndex, int s2, int s3);
int mapValue(int x, int in_min, int in_max, int out_min, int out_max);


void setup() {
  Serial.begin(57600); // initialize serial connection for debugging
  Arm.attach(25);
  Gripper.attach(24);
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

  //MoveStart(); testing function
}

void loop() {
  // Get the sensor data
  Serial.println(' ');
  readColorSensors();
  ultrasonicRead();

  // move the arm to be pointing up to move out of the way
  Arm.write(100);
  Gripper.write(40);

  //////////////////////// OBJECT AVOIDANCE ALGORITHM /////////////////////////////
    // Drive algotithm for the robot to follow the wall
    // to start, the robot will move forward until it detects a wall
    //   - During this phase it will store x_floating into x_pos to determine which half it is in
    //   - We do this to only read the data when the robot is moving forward and dont change the stored value while it is moving either left or right
    //   - If we didn't do this, the robot will change directions while moving left or right
    // once it detects a wall, it will move right or left depending on which half of the maze it is in
    //   - i.e. if x_pos<18, the robot will move right, else it will move left
    //   - The robot has what I call robot units while it moves, the maze has a width of 35 robot units and a height of 140 robot units
    // as the robot moves left it will decrease the x_floating, ad it moves right it will increase x_floating
    //   - this is needed to store the x position of the robot in space
    //   - we store this in a different value because we dont want the robot to think based on its changing x_position,
    //     we want to only store the initial position on the wall
    //   - if the robot reaches a bound before detecting a hole, it will invert its direction to go the other way
    // if the robot detects a hole, it will continue moving left or right before moving forward to adjust its delay
    //   - this is to prevent the robot from moving forward immediately after not detecting a wall
    // repeate the process until the y_position is at its limit, if this occours, stop the robot.

  if (distance > 20){ // if we dont see a wall 
    if (delayCondition == 1){ // check if we were moving right before and delay only once when reached
      delay(500);
      if (move_val == 1){
        x_floating = x_floating+5; // adjust the x_position to accomidate for the delay
      } else if (move_val == -1){
        x_floating = x_floating-5; // adjust the x_position to accomidate for the delay
      }
      delayCondition = 0;
    }
    y_pos++; // increase the y_position as it moves forward
    moveMotorsForward();
    x_pos = x_floating; // store the x_floating into the stable x_position
  } else {
    delayCondition = 1; // set the delay condition for the hole
    if (distance<10){
      moveMotorsBackward(); // if it goes to close to the wall, move back a bit
    } else if (x_pos < 18){
      if(x_floating<34){ // if it is not at a bound, move in the right
        move_val = 1;
        moveMotorsRight();
        x_floating++; // increase x_position
      } else {
        x_pos = x_floating; // if we reach the right bound, store x_floating in x_position to initialize a reset to the x_position
      }
    } else if (x_pos > 17){
      if(x_floating>0){ // if it is not at a bound, move in the left
        move_val = -1; 
        moveMotorsLeft();
        x_floating--; // decrease x_position
      } else {
        x_pos = x_floating; // if we reach the left bound, store x_floating in x_position to initialize a reset to the x_position
      }

    }
  }
  if(y_pos > 140){ // if we reach the end of the maze, stop the robot
    stopMotors();
    delay(10000);
  }
  /////////////////////////////////////////////////////////////////////////////
  delay(100); // clock cycle to sync the robot units
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// MOTOR DEFINITIONS /////////////////////////////////
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
    // Set motor speeds
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, HIGH);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsBackward() {
  Serial.println("Backward");
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL+12);
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotorsRight() {
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL_MOTOR2*1.9);
  analogWrite(MOTOR3_PWM, PWM_VAL);
  Serial.println("Right");
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, HIGH);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsLeft() {
  Serial.println("Left");
    // Set motor speeds
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL_MOTOR2*1.9);
  analogWrite(MOTOR3_PWM, PWM_VAL);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotorsForwardRight() {
  Serial.println("Forward Right");
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL-5);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, HIGH);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsForwardLeft() {
  Serial.println("Forward Left");
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL-5);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

void moveMotorsBackwardRight() {
  Serial.println("Backward Right");
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL-5);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotorsBackwardLeft() {
  Serial.println("Backward Left");
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL-5);
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
}

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

void moveMotors180() {
  Serial.println("spinning");
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL);
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

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

void readColorSensors(){
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

    //Serial.println(' ');
    // Determine the color based on the mapped values
    if (mappedRed < 65 && mappedGreen < 65 && mappedBlue < 65) {
      colorsensorValues[i] = YELLOW;
      Serial.println("Color Est: Yellow");
    } else if (mappedRed > 100 && mappedGreen > 160 && mappedBlue > 100){
      colorsensorValues[i] = BLACK;
      Serial.println("Color Est: Black");
    } else if (mappedRed < mappedBlue && mappedRed < mappedGreen) {
      colorsensorValues[i] = RED;
      Serial.println("Color Est: Red");
    } else if (mappedBlue < mappedRed && mappedBlue < mappedGreen) {
      colorsensorValues[i] = BLUE;
      Serial.println("Color Est: Blue");
    } else if (mappedGreen < mappedRed && mappedGreen < mappedBlue) {
      colorsensorValues[i] = GREEN;
      Serial.println("Color Est: Green");
    } else {
      Serial.println("Color not recognized");
      colorsensorValues[i] = 0;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// END ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
