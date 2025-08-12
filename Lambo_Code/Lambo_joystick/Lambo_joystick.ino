// Pin connections for Motor A
#define MOTOR1_PWM 0
#define MOTOR1_CWISE 1
#define MOTOR1_ANTI_CWISE 2

// Pin connections for Motor B
#define MOTOR2_PWM 3
#define MOTOR2_CWISE 4
#define MOTOR2_ANTI_CWISE 5

// Pin connections for Motor C
#define MOTOR3_PWM 6
#define MOTOR3_CWISE 7
#define MOTOR3_ANTI_CWISE 8

// Motor Speed (PWM) value between 150 and 255
#define PWM_VAL 150 

// Define the analog input pins for the joystick axes
const int joystickXPin = A8; // Pin number used for xValue
const int joystickYPin = A7; // Pin number used for yValue

const int joystickint = 500;
const int joystickdeadzone = 100;

int case_num = 1;

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
void moveMotorsSpinDrive();

void setup() {
  Serial.begin(9600); // initialize serial connection for debugging

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
  pinMode(joystickXPin, INPUT);
  pinMode(joystickYPin, INPUT);

  // Initial state - Turn off all the motors
  stopMotors();
  //moveTest();
}

void loop() {
  unsigned long xPosition = analogRead(joystickXPin);
  unsigned long yPosition = analogRead(joystickYPin);
  Serial.println(xPosition);
  Serial.println(yPosition);

  // Set motor speeds
  analogWrite(MOTOR1_PWM, PWM_VAL);
  analogWrite(MOTOR2_PWM, PWM_VAL);
  analogWrite(MOTOR3_PWM, PWM_VAL);

  // Control direction based on joystick position
  if (yPosition > 612 && xPosition > 612) {
    moveMotorsForwardRight();
  } else if (yPosition > 612 && xPosition < 412) {
    moveMotorsForwardLeft();
  } else if (yPosition < 412 && xPosition > 612) {
    moveMotorsBackwardRight();
  } else if (yPosition < 412 && xPosition < 412) {
    moveMotorsBackwardLeft();
  } else if (yPosition > 612) {
    moveMotorsForward();
  } else if (yPosition < 412) {
    moveMotorsBackward();
  } else if (xPosition < 412) {
    moveMotorsRight();
  } else if (xPosition > 612) {
    moveMotorsLeft();
  } else {
    stopMotors();
  }
  delay(100);
}

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
  analogWrite(MOTOR2_PWM, adjustedSpeed * 1.75); // Double the speed of motor 2 and adjust the other motors to the adjustedSpeed
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
  analogWrite(MOTOR2_PWM, adjustedSpeed * 1.75); // Double the speed of motor 2 and adjust the other motors to the adjustedSpeed
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
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, HIGH);
  digitalWrite(MOTOR2_ANTI_CWISE, LOW);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
}

void moveMotorsForwardLeft() {
  Serial.println("Forward Left");
  digitalWrite(MOTOR1_CWISE, HIGH);
  digitalWrite(MOTOR1_ANTI_CWISE, LOW);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, LOW);
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

void moveMotorsSpinDrive() {
  switch(case_num) {
        case 1:
            moveMotorsForward();
            break;
        case 2:
            moveMotorsForwardLeft();
            break;
        case 3:
            moveMotorsBackwardLeft();
            break;
        case 4:
            moveMotorsBackward();
            break;
        case 5:
            moveMotorsBackwardRight();
            break;
        case 6:
            moveMotorsForwardRight();
            break;
        default:
            printf("Default case, no match found.\n");
            case_num = 0;
  }
  case_num++;
  Serial.println(case_num);
  delay(300);
  Serial.println("spinning forward");
  digitalWrite(MOTOR1_CWISE, LOW);
  digitalWrite(MOTOR1_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR2_CWISE, LOW);
  digitalWrite(MOTOR2_ANTI_CWISE, HIGH);
  digitalWrite(MOTOR3_CWISE, LOW);
  digitalWrite(MOTOR3_ANTI_CWISE, HIGH);
  delay(320);
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
