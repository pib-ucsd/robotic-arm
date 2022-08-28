#include <MPU6050_light.h>
#include <Wire.h>
#include <Servo.h>

// ARM LENGTH CONSTANTS
#define SHOULDER_LENGTH 5 // shoulder/short length (screw to screw)
#define FOREARM_LENGTH 8 // long length + 1/2 gripper (screw to screw)

// GRIPPER POSITION CONSTRAINTS
#define MAX_RADIUS sqrt(SHOULDER_LENGTH * SHOULDER_LENGTH + FOREARM_LENGTH * FOREARM_LENGTH)
#define MIN_X FOREARM_LENGTH
#define MAX_X MAX_RADIUS
#define MIN_Y FOREARM_LENGTH
#define MAX_Y MAX_RADIUS

// MOVEMENT THRESHOLD AND SCALE
#define MOVEMENT_SCALE 50
#define ANGLE_THRESHOLD 35
#define CART_SPEED 20

// PIN DEFINITIONS
#define SLIDER_POT A3
#define RIGHT_BUTTON 2
#define CART_SERVO 5
#define LEFT_BUTTON 4
#define SHOULDER_SERVO 9
#define ELBOW_SERVO 10
#define GRIPPER_SERVO 11
#define RESET_BUTTON 12
#define CALIBRATING_LED 13

// OBJECTS
MPU6050 mpu(Wire);
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;
Servo cartServo;

// GLOBAL VARIABLES
float gripperXPos = (MIN_X+MAX_X)/2;
float gripperYPos = (MIN_Y+MAX_Y)/2;
float angX = 0;
float angY = 0;
float angZ = 0;
float shoulderAngle;
float elbowAngle;
int gripperRot = 0;
int slidePos = 0;
int cartSpeed = 0;
bool goRight = false;
bool goLeft = false;

unsigned long timer = 0;

void setup() {
  // GENERAL SETUP (pinModes and attachments)
  Serial.begin(9600);
  pinMode(CALIBRATING_LED, OUTPUT);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(SLIDER_POT, INPUT);
  shoulderServo.attach(SHOULDER_SERVO);
  elbowServo.attach(ELBOW_SERVO);
  gripperServo.attach(GRIPPER_SERVO);

  // CART SETUP
  // Write before attach to define starting state (stopped)
  // 0 - turn one way, 180 - other way, 90 - stop
  cartServo.write(90);
  cartServo.attach(CART_SERVO);
  
  // SENSOR SETUP
  Wire.begin();
  mpu.begin();
  calibrate();
}

void loop() {
  checkReset();
  readSensors();
  determineMovement();
  controlArm();
  controlGripper();
  controlCart();
  printData();
}

// CALL ON CALIBRATE METHOD FROM MPU LIBRARY
void calibrate() {
  digitalWrite(CALIBRATING_LED, HIGH);
  mpu.calcGyroOffsets();
  digitalWrite(CALIBRATING_LED, LOW);
}

// CALL ON CALIBRATE IF RESET BUTTON PRESSED
void checkReset() {
  if(!digitalRead(RESET_BUTTON)) {
    cartServo.write(90);
    gripperXPos = (MIN_X+MAX_X)/2;
    gripperYPos = (MIN_Y+MAX_Y)/2;
    cartSpeed = 0; // 0 for stopped, CART_SPEED for right, -CART_SPEED for left
    calibrate();
  }
}

// READ ANGLES FROM MPU6050, ADJUST SIGN DEPENDING ON MPU ORIENTATION
// READ SLIDE POTENTIOMETER FOR GRIPPER
// READ BUTTONS TO DETERMINE WHERE TO MOVE CART
void readSensors() {
  mpu.update();
  angX = mpu.getAngleX();
  angY = mpu.getAngleY();
  angZ = -mpu.getAngleZ();

  slidePos = analogRead(SLIDER_POT);
  goRight = !digitalRead(RIGHT_BUTTON);
  goLeft = !digitalRead(LEFT_BUTTON);
}

// CALCULATE XY-COORDINATES BASED ON CALCULATED ANGLES
// MAP SLIDER POTENTIOMETER RAW VALUE
void determineMovement() {
  gripperXPos = constrain(gripperXPos + floor(angX / ANGLE_THRESHOLD) / MOVEMENT_SCALE, MIN_X, MAX_X);
  gripperYPos = constrain(gripperYPos + floor(angY / ANGLE_THRESHOLD) / MOVEMENT_SCALE, MIN_Y, MAX_Y);

  gripperRot = constrain(map(slidePos, 100, 750, 0, 120), 0, 120);
}

// SET CART SPEED BASED ON MOVEMENT BUTTON PRESSED
void controlCart() {
  if(goRight) {
    cartSpeed = 90 + CART_SPEED;
    cartServo.write(cartSpeed);
  }
  else if(goLeft) {
    cartSpeed = 90 - CART_SPEED;
    cartServo.write(cartSpeed);
  }
  else {
    cartSpeed = 90;
    cartServo.write(cartSpeed);
  }
  delay(10);
  return;
}

// WRITE THE CALCULATED ANGLE TO THE GRIPPER SERVO
void controlGripper() {
  gripperServo.write(gripperRot);
  delay(10);
  return;
}

// PRINT CURRENT EXPECTED CART SPEED
void printData() {
  if((millis()-timer)>10) // print data every 10ms
  {
    Serial.print("cart speed: ");
    Serial.print(cartSpeed);
    Serial.println();
    timer = millis();
  }
}

// CALCULATE THE INVERSE KINEMATICS BASED ON XY-COORDINATES AND WHICH SERVO
float inverseKinematics(float x, float y, byte servo_num) {
  double cb = (-sq(SHOULDER_LENGTH) - sq(FOREARM_LENGTH) + sq(x) + sq(y)) / (2 * (SHOULDER_LENGTH) * (FOREARM_LENGTH));
  double sbin = 1 - (sq(cb));
  double sb = sqrt(sbin);
  if (servo_num == SHOULDER_SERVO) {
    long double ca = ((x*SHOULDER_LENGTH)+(x*FOREARM_LENGTH*cb)+(y*FOREARM_LENGTH*sb))/(sq(x)+sq(y));
    double sain = 1 - (sq(ca));
    double sa = sqrt(sain);
    return (atan2(sa,ca)) * (180/PI);
  }
  else if (servo_num == ELBOW_SERVO) { 
    return (atan2(sb,cb)) * (180/PI);
  }
}

// MOVE SERVO POSITIONS TO CALCULATED ANGLES
void controlArm() {
  shoulderAngle = constrain(180-2*(inverseKinematics(gripperXPos, gripperYPos, SHOULDER_SERVO)),0,180);
  elbowAngle = 180-constrain(inverseKinematics(gripperXPos, gripperYPos, ELBOW_SERVO),0,180);
  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  return;
}
