# Robotic Arm


## **PART 1: CONCEPTS**

### Inverse Kinematics
### Circuits
### MPU6050
### PWM
### I2C

## **PART 2: SYSTEM DESIGN**

## **PART 3: CIRCUIT**

## **PART 4: BUILD**

## **PART 5: CODE**

### **Required Libraries**
* *MPU6050_light*
* *Wire.h*
* *Servo.h*
### **Reading data from the MPU6050**
The following lines of code can be used to connect and read data from the MPU6050

    #include <MPU6050_light.h>
    #include <Wire.h>
    MPU6050 mpu(Wire); // Create MPU6050 object called mpu

    float angleX;
    float angleY;
    float angleZ;

    void setup() {
        // Connect to the MPU6050
        Wire.begin();
        mpu.begin();

        // Calculate offsets to more accurately determine absolute MPU6050 angles
        mpu.calcGyroOffsets();
    }

    void loop() {
        readFromMPU();
    }

    void readFromMPU() {
        mpu.update();
        angX = mpu.getAngleX();
        angY = mpu.getAngleY();
        angZ = mpu.getAngleZ();
    }

### **Converting Angles to XY Coords**
The following link contains code as well as a simulation of mapping angles read from the MPU6050 to the XY coordinate system

[Converting Angles to XY Coords on WOKWI](https://wokwi.com/projects/335575380850115156)

### **Implementing Inverse Kinematics in Code**
The following function can be used to calculate the inverse kinematics of a two joint system

**Note:** Please refer to how the shoulder and forearm joints/limbs are defined in the concepts section

    float inverseKinematics(float x, float y, bool isShoulder) {
        double cb = (-sq(SHOULDER_LENGTH) - sq(FOREARM_LENGTH) + sq(x) + sq(y)) / (2 * (SHOULDER_LENGTH) * (FOREARM_LENGTH));
        double sbin = 1 - (sq(cb));
        double sb = sqrt(sbin);
        if (isShoulder) {
            long double ca = ((x * SHOULDER_LENGTH) + (x * FOREARM_LENGTH * cb) + (y * FOREARM_LENGTH * sb)) / (sq(x) + sq(y));
            double sain = 1 - (sq(ca));
            double sa = sqrt(sain);
            return (atan2(sa,ca)) * (180/PI);
        }
        else { 
            return (atan2(sb,cb)) * (180/PI);
        }
    }

### **Converting XY Coordinates into servo angles with Inverse Kinematics**
The following link adds the inverse kinematics code into the previous codebase to convert the XY Coordinates into servo angles *(Simulation with servos included)*

[Converting XY Coords into Servo Angles on WOKWI](https://wokwi.com/projects/335575417969705556)

### **Reading the Slide Potentiometer**
The following code is an example of how to read the data from a slide potentiometer

**Note:** If data input is analog, pinMode must be `INPUT` and not `INPUT_PULLUP`

    #define SLIDE_POT_PIN A0

    int slidePotReading;

    void setup() {
        pinMode(SLIDE_POT_PIN, INPUT);
    }

    void loop() {
        slidePotReading = analogRead(SLIDE_POT_PIN); // 0-1023
    }

### **Set Gripper Servo Angle with Slide Potentiometer**
The following link combines the previous code with a new segment that converts the readings from the slide potentiometer to control the angle of the gripper servo *(Simulation included)*

[Mapping Slide Potentiometer to Gripper Servo Angle on WOKWI](https://wokwi.com/projects/335575464075592276)

### **Driving Continuous Servos**
Assuming that the servos (if applicable) are properly calibrated, the FS90R continuous servos have angle values of `90` for stopped, `180` for maximum speed in one direction and `0` for maximum speed in the other direction. You can connect the data pin of the servo to an PWM pin on the Arduino and use the standard method of writing to angles to Servos to drive the FS90R.

In this project, we chose to use buttons to control the servos to move in either direction. The following link contains the addition of the continous servo controlled by two additional buttons and is the full code necessary for this project.

[Press Buttons to Move Servos on WOKWI](https://wokwi.com/projects/337906127494709842)

## [**FULL CODE**](https://wokwi.com/projects/335575481692717652)
The full code necessary for this project is linked in `FULL CODE` heading above and is accompanied by the circuitry to simulate this project on WOKWI.
