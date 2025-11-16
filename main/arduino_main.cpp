// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"
#include <QTRSensors.h> // line sensor library
#include <ESP32SharpIR.h> // ir sensor library
#include <Wire.h>
#include <Arduino_APDS9960.h> // color sensor library
#include <bits/stdc++.h>
#include <ESP32Servo.h>


#define ONBOARD_LED_PIN 2 // LED pin

#define IN1 19 // right motor pins
#define IN2 18

#define IN3 17 // left motor pins
#define IN4 16  

#define IN5 4 // launch motor pins
#define IN6 5

const uint8_t LINE_FOLLOW_PINS[] = {36, 35, 34, 14, 13, 39, 33, 32}; // line sensor pins

#define LEFT_IR_PIN 0 // ir pin
#define FRONT_IR_PIN 25 // ir pin
#define RIGHT_IR_PIN 26 // ir pin

#define APDS9960_INT_PIN 0 // color pins
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define ANGLE_SERVO_PIN 23 // angle servo pin


const char* const MODES[] = {"Manual", "Color automation", "Wall automation", "Line automation"};
const uint8_t TOP_MOTOR_SPEED = 255;

const uint8_t MANUAL = 0;
const uint8_t COLOR_AUTOMATION = 1;
const uint8_t WALL_AUTOMATION = 2;
const uint8_t LINE_AUTOMATION = 3;

const int8_t NONE = -1;

uint8_t currentMode = 0; // current mode for robot
extern ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // controller



//------------------------------------------------------------------------------------------------//
//------------------------------------------<< HELPERS >>-----------------------------------------//
//------------------------------------------------------------------------------------------------//

/*
 * Cleans terminal to debug easier.
 */
void cleanTerminalSimple() {
    for (uint8_t i = 0; i < 25; i++) {
        Console.println();
    }
}

/*
 * Alternative way to clean easier.
 * Removes current line.
 */
void cleanTerminalAdvancedSeq1() {
    Console.print('\r');
    Console.print("\x1B[2K");
}

/*
 * Sets cursor back to beginning for next clear call.
 */
void cleanTerminalAdvancedSeq2() {
    Console.write('\r');
}

/*
 * Sets current mode based on controller input.
 * @param ctl pointer to controller
 */
void setMode(ControllerPtr ctl) {
    if (ctl->b()) {
        currentMode = MANUAL;
    } else if (ctl->a()) {
        currentMode = COLOR_AUTOMATION;
    } else if (ctl->x()) {
        currentMode = WALL_AUTOMATION;
    } else if (ctl->y()) {
        currentMode = LINE_AUTOMATION;
    }
}

//------------------------------------------------------------------------------------------------//
//---------------------------------------<< MOTOR MOVEMENT >>-------------------------------------//
//------------------------------------------------------------------------------------------------//

const uint16_t MAX_JOYSTICK_INPUT = 512;


void moveMotorsHelper(int leftSpeedForward, int leftSpeedBackward, int rightSpeedForward, 
    int rightSpeedBackward);


/*
 * Sets up the motor pins as outputs.
 */
void motorSetup() {
    pinMode(IN1, OUTPUT); // left motor
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); // right motor
    pinMode(IN4, OUTPUT);
}

/*
 * Handles the movement of the robot.
 * Left joystick controls left motor and right joystick controls right motor.
 * @param ctl pointer to controller
 */
void moveMotorsAdvanced(ControllerPtr ctl) {
    int16_t lY = ctl->axisY(); // left joystick Y axis
    uint8_t aLY = abs(lY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // adjusted left joystick 
                                                      // Y axis for speed input [0, TOP_MOTOR_SPEED]
    int16_t rY = ctl->axisRY();// right joystick Y axis
    uint8_t aRY = abs(rY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // adjusted right joystick Y axis 
                                                             // for speed input [0, TOP_MOTOR_SPEED]

    bool lForward = (lY <= -200); // determine if left joystick is moving forward
    bool lBackward = (lY >= 200); // determine if left joystick is moving backwards
    bool rForward = (rY <= -200);
    bool rBackward = (rY >= 200);

    if (lForward) { // if left joystick is forward, move motor forward at adjusted speed
        analogWrite(IN1, aLY);
        analogWrite(IN2, 0);
    } else if (lBackward) {
        analogWrite(IN1, 0);
        analogWrite(IN2, aLY);
    } else { // stop
        analogWrite(IN1, 0);
        analogWrite(IN2, 0);
    }

    if (rForward) { // if right joystick is forward, move motor forward at adjusted speed
        analogWrite(IN3, aRY);
        analogWrite(IN4, 0);
    } else if (rBackward) {
        analogWrite(IN3, 0);
        analogWrite(IN4, aRY);
    } else { // stop
        analogWrite(IN3, 0);
        analogWrite(IN4, 0);
    }
}

/*
 * Handles the movement of the robot.
 * Left joystick controls both motors and the right joystick turns it to one direction.
 * @param ctl pointer to controller
 */
void moveMotorsSimple(ControllerPtr ctl) {
    int16_t lY = ctl->axisY(); // left joystick y-value
    uint8_t aLY = abs(lY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // adjusted left joystick Y axis 
                                                             // for speed input [0, TOP_MOTOR_SPEED]
    int16_t rX = ctl->axisRX(); // right joystick x-value
    uint8_t aRX = abs(rX) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // adjusted right joystick X axis 
                                                             // for speed input [0, TOP_MOTOR_SPEED]
    bool lB = ctl->l1(); // left bumper
    bool rB = ctl->r1(); // right bumper

    uint8_t motorAdjustment = max(0, aLY - aRX); // adjusts motor speed for either side
    bool forward = (lY <= -200); // determine if left joystick is moving forward
    bool backward = (lY >= 200);
    bool left = (rX <= -30); // determine if right joystick is pushed left
    bool right = (rX >= 30);

    if (forward) {
        if (left) {
            moveMotorsHelper(motorAdjustment,0, aLY, 0);
        } else if (right) {
            moveMotorsHelper(aLY, 0, motorAdjustment, 0);
        } else {
            moveMotorsHelper(aLY, 0, aLY, 0);
        }
    } else if (backward) {
        if (left) {
            moveMotorsHelper(0, motorAdjustment, 0, aLY);
        } else if (right) {
            moveMotorsHelper(0, aLY, 0, motorAdjustment);
        } else {
            moveMotorsHelper(0, aLY, 0, aLY);
        }
    } else { // left joystick netural (within -200, 200)
        if (lB) { // spin counter-clockwise
            moveMotorsHelper(0, TOP_MOTOR_SPEED, TOP_MOTOR_SPEED, 0);
        } else if (rB) { // spin clockwise
            moveMotorsHelper(TOP_MOTOR_SPEED, 0, 0, TOP_MOTOR_SPEED);
        } else { // standstill
            moveMotorsHelper(0, 0, 0, 0);
        }
    }
}

/*
 * Prints all button values for the controller.
 * @param ctl pointer to controller
 */
void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "DPAD: %2d A: %2d B: %2d X: %2d Y: %2d LX: %4d LY: %4d RX: %4d RY: %4d L1: %2d R1: %2d \
        L2: %2d R2: %2d\n",
        ctl->dpad(),        // D-pad
        ctl->a(),           // Letter buttons
        ctl->b(),
        ctl->x(),
        ctl->y(),
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->l1(),           // Bumpers
        ctl->r1(),
        ctl->l2(),
        ctl->r2()
    );
}

/*
 * Helper to simplify motor control.
 * Takes in four values: left speed f/b, and right speed f,b
 * @param leftSpeedForward left motor speed forward
 * @param leftSpeedBackward left motor speed backwards
 * @param rightSpeedForward right motor speed forward
 * @param rightSpeedBackward right motor speed backwards
 */
void moveMotorsHelper(int leftSpeedForward, int leftSpeedBackward, int rightSpeedForward, 
    int rightSpeedBackward) {
    analogWrite(IN1, leftSpeedForward);
    analogWrite(IN2, leftSpeedBackward);
    analogWrite(IN3, rightSpeedForward);
    analogWrite(IN4, rightSpeedBackward);
}

//------------------------------------------------------------------------------------------------//
//---------------------------------------<< MOTOR LAUNCHER >>-------------------------------------//
//------------------------------------------------------------------------------------------------//

const uint8_t MIN_SPEED = 55;

bool launchMotor = false;
bool launchMotorRamped = false;

float speedMultiplier = 0.67; // range is .22 to 1.0
uint8_t maxLauncherSpeed = constrain((int)(TOP_MOTOR_SPEED * speedMultiplier), 55, 255);

int rampTimeMillis = 1000;


void rampUpLaunchMotor();
void rampDownLaunchMotor();
void moveLaunchMotorHelper(int speedForward, int speedBackward);


/*
 * Setup for launch motor.
 */
void launchMotorSetup() {
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
}

/* 
 * Checks to see if launch motors need to be ramped up/down or neither.
 * @param ctl pointer to controller
 */
void checkLaunchMotor(ControllerPtr ctl) {
    launchMotor = (ctl->r2()) ? true : false;

    if (launchMotor && !launchMotorRamped) {
        rampUpLaunchMotor();
        launchMotorRamped = true;
    } else if (!launchMotor && launchMotorRamped) {
        rampDownLaunchMotor();
        launchMotorRamped = false;
    }
}


/*
 * Ramp launch motors up so it doens't reach max speed instantly.
 */
void rampUpLaunchMotor() {
    int numSteps = maxLauncherSpeed - MIN_SPEED;

    for (int i = MIN_SPEED; i <= numSteps; i++) {
        moveLaunchMotorHelper(i, 0);
        delay(rampTimeMillis / numSteps);
    }
}

/*
 * Ramp down launch motors so it doesn't stop instantly.
 */
void rampDownLaunchMotor() {
    uint8_t initialSpeed = maxLauncherSpeed;
    int numSteps = initialSpeed - MIN_SPEED;

    for (uint8_t i = initialSpeed; i >= MIN_SPEED; i--) {
        moveLaunchMotorHelper(i, 0);
        delay(rampTimeMillis / numSteps);
    }

    moveLaunchMotorHelper(0, 0);
}

/*
 * Helper to simplify launch motor control.
 * @param speedForward motor speed forward
 * @param speedBackward motor speed backward
 */
void moveLaunchMotorHelper(int speedForward, int speedBackward) {
    analogWrite(IN5, speedForward);
    analogWrite(IN6, speedBackward);
}

//------------------------------------------------------------------------------------------------//
//-------------------------------------------<< SERVO >>------------------------------------------//
//------------------------------------------------------------------------------------------------//

Servo angleServo;


const uint8_t ANGLE_CLOSED = 0;
const uint8_t ANGLE_OPEN = 60;

uint8_t angle = 0; // current angle of angleServo


/*
 * Setups up the angle and collection servo.
 */
void angleServoSetup() {
    angleServo.attach(ANGLE_SERVO_PIN);
    angle = angleServo.read(); // current angle
}

/*
 * Handles the movement of the angle servo.
 * Dpad up and down control the angle of the servo. (N U D R L) -> (0, 1, 2, 4, 8).
 * @param ctl pointer to controller
 */
void moveAngleServo(ControllerPtr ctl) {
    uint8_t dpad = ctl->dpad();

    if (dpad == DPAD_UP) {
        angleServo.write(ANGLE_OPEN);
    } else if (dpad == DPAD_DOWN) {
        angleServo.write(ANGLE_CLOSED);
    }
}

//------------------------------------------------------------------------------------------------//
//--------------------------------------<< LINE AUTOMATION >>-------------------------------------//
//------------------------------------------------------------------------------------------------//

QTRSensors qtr;
const uint8_t NUM_LINE_SENSORS = sizeof(LINE_FOLLOW_PINS) / sizeof(LINE_FOLLOW_PINS[0]);
uint16_t lineArray[NUM_LINE_SENSORS];


float kP= 0.5; // proportional constant
float kD = 10.0; // derivative constant
float kI = 0; // integral constant
float lastEror = 0; // last error for derivative calculation
float sum = 0; // sum of errors for integral calculation

bool pressedPU = false;
bool pressedPD = false;
bool pressedDU = false;
bool pressedDD = false;

int16_t lineThreshold = 950;
uint8_t lineSpeed = TOP_MOTOR_SPEED;
uint8_t ricochet = 0;
int8_t lastRemembered = -1;
uint8_t maxRicochet = 10;

bool lineCalibrated = false;


void calibrateLineSensors();
float lineHelper();
void updatePID(ControllerPtr ctl);
void setPressed(bool val1, bool val2, bool val3, bool val4);
bool checkInBetween();


/*
 * Sets up the line following sensors.
 */
void lineSetup() {
    qtr.setTypeAnalog();
    qtr.setSensorPins(LINE_FOLLOW_PINS, NUM_LINE_SENSORS);
}

/*
 * Fabian's Algorithm
 * Handles the line following automation.
 * @param ctl pointer to controller
 */
void lineAutomationA(ControllerPtr ctl) {
    updatePID(ctl);
    uint8_t speed = TOP_MOTOR_SPEED;
    float correction = lineHelper(); // correction for the lineHelper that will fix the robot's path
    correction = constrain(correction, -speed, speed);
    uint8_t leftSpeed = constrain((int)(speed + correction), 0, TOP_MOTOR_SPEED);
    uint8_t rightSpeed = constrain((int)(speed - correction), 0, TOP_MOTOR_SPEED);
    moveMotorsHelper(leftSpeed ,0 ,rightSpeed ,0); // move motors with correction applied to speed
}

/*
 * Handles the line following automation.
 * Three cases:
 *     1. no sensor activated -> move forward
 *     2. first sensor activated -> move left
 *     3. last sensor activated -> move right
 *     4. no sensor activated -> spin in certain direction
 */
void lineAutomationB() {
    uint16_t firstSensor = lineArray[NUM_LINE_SENSORS - NUM_LINE_SENSORS];
    uint16_t lastSensor = lineArray[NUM_LINE_SENSORS - 1];
    bool firstOnLine = firstSensor > lineThreshold;
    bool lastOnLine = lastSensor > lineThreshold;
    bool lineInBetween = checkInBetween();
    
    if (firstOnLine && ricochet < maxRicochet) { // line on first -> turn right
        moveMotorsHelper(0, lineSpeed, lineSpeed, 0);
        ricochet++;
        lastRemembered = 0;
    } else if (lastOnLine && ricochet < maxRicochet) { // line on last -> turn left
        moveMotorsHelper(lineSpeed, 0, 0, lineSpeed);
        ricochet++;
        lastRemembered = 1;
    } else if (lineInBetween || ricochet >= maxRicochet) { // on line/ricochet -> move forward
        moveMotorsHelper(lineSpeed, 0, lineSpeed, 0);
        ricochet = 0;
    } else { // nothing sensed -> turn in last known direction

        if (lastRemembered == 0) { // last remembered is first
            moveMotorsHelper(0, 0, lineSpeed, 0);
        } else if (lastRemembered == 1) { // last remembered is last
            moveMotorsHelper(lineSpeed, 0, 0, 0);
        }

        ricochet = 0;
    }
}

/*
 * Checks if the line is present betwen the 2nd and 7th sensor.
 * @return true if line is present, else false.
 */
bool checkInBetween() {
    for (uint8_t i = 1; i < NUM_LINE_SENSORS - 1; i++) {

        if (lineArray[i] > lineThreshold) {
            return true;
        }
    }

    return false;

}

/*
 * Prints the line sensor values.
 */
void lineDebug() {
    for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++) {
        Console.printf("%d  ", lineArray[i]);
    }
}

/*
 * Updates lineArray with new values.
 */
void updateLine() {
    qtr.readLineBlack(lineArray);
}

/*
 * Helper method that calculates error and PID.
 * @return how much to correct using PID
 */
float lineHelper() {
    float error = 0;
    int wPosition = 0;
    int16_t weights[] = {-3000, -2000, -1000, -500, 500, 1000, 2000, 3000}; // Adjust weights based 
                                                                            // on number of sensors

    for(uint8_t i = 0; i < NUM_LINE_SENSORS; i++) { // for loop to go through all sensors
        wPosition += lineArray[i] * weights[i];   // calculates the weighted error with the value of   
    }     // the sensor. Example: if leftmost sensor is on black (1000) and all others are white (0)
    // then error = 0 * -3 + 1,000 * -2 +  1,000 * -1 + 0 * 0 + 0* 0 + 0 * 1 + 0 * 2 + 0 * 3 = -3000
    error = wPosition / 3000; // Normalize error by diving by the highest sensor value that is 1000;
    float P = kP * error; // equation for the porpotional term for the immediate correction
    float D = kD * (error - lastEror); // equation for the derivative term for appliying a small 
                                       // brake on the porpotional term
    sum = sum + error; // sum of all errors up to now for integral term to use to track long term
                       // errors over the course
    float I = kI * sum; // equation for the integral term to correct for long term error and apply 
                        // small amount of correction
                      // The integral term is very small and only affects the robot if a long and
                      // difficult course.
    lastEror = error; // Store current error for the next calculation for the derivative term 
    return P + D + I; // return the added up corrections into one value
}

/*
 * Calibrates line sensors over 250 iterations. 
 * Increase if need be.
 */
void calibrateLineSensors() {
    int16_t numSteps = 250;
    uint8_t calibrateDelayMillis = 20;

    for (uint8_t i = 0; i < numSteps; i++) {
        qtr.calibrate();
        delay(calibrateDelayMillis);
    }
}

/*
 * All in one sequence to complete calibration.
 */
void startCalibration() {
    moveMotorsHelper(TOP_MOTOR_SPEED, 0, 0, TOP_MOTOR_SPEED);
    calibrateLineSensors();
    moveMotorsHelper(0, 0, 0, 0);
    lineCalibrated = true;
    currentMode = MANUAL;
}

/*
 * Updates PID values remotely using buttons. Easy switching.
 * @param ctl pointer to controller 
 */
void updatePID(ControllerPtr ctl) {
    uint8_t dpad = ctl->dpad();

    if (dpad == DPAD_UP && !pressedPU) {
        kP++;
        setPressed(true, false, false, false);
    } else if (dpad == DPAD_DOWN && !pressedPD) {
        kP--;
        setPressed(false, true, false, false);
    } else if (dpad == DPAD_RIGHT && !pressedDU) {
        kD++;
        setPressed(false, false, true, false);
    } else if (dpad == DPAD_LEFT && !pressedDD) {
        kD--;
        setPressed(false, false, false, true);
    } else {
        setPressed(false, false, false, false);
    }
}

/*
 * Ensures tapping a button doesn't increase value significantly.
 */
void setPressed(bool val1, bool val2, bool val3, bool val4) {
    pressedPU = val1;
    pressedPD = val2;
    pressedDU = val3;
    pressedDD = val4;
}


//------------------------------------------------------------------------------------------------//
//-------------------------------------<< COLOR AUTOMATION >>-------------------------------------//
//------------------------------------------------------------------------------------------------//

TwoWire I2C_0 = TwoWire(0);// color
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT_PIN);
const int I2C_FREQUENCY = 100000; // I2C frequency for color sensor


const uint8_t NUM_COLORS = 4;
const uint8_t RED = 0;
const uint8_t GREEN = 1;
const uint8_t BLUE = 2;
const uint8_t ALPHA = 3;

float redAdjust = 1.43;
float greenAdjust = 1.25;

int colorArray[NUM_COLORS]; // array holding information for r, g, b, a

int8_t sampleColor = NONE; // saves red, green, or blue (no other colors)
int8_t currentColor = NONE;

bool checkInitial = false; // value to check if we moved off the color
bool colorFound = false; // value to check if we found the color again
bool sampled = false;

uint8_t redInARow = 0;
uint8_t greenInARow = 0;
uint8_t blueInARow = 0;
uint8_t rangeThreshold = 10;
uint8_t inARowThreshold = 3;


int recordColorOverCheck();
int recordColorMultiplierCheck();
int recordColorThresholdCheck();
int recordColorRecordedCheck();
void resetColorVariables();
void updateInARow(int val1, int val2, int val3);

/*
 * Sets up the color sensor and I2C communication.
 */
void colorSetup() {
    I2C_0.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
    apds.setInterruptPin(APDS9960_INT_PIN);
    apds.begin();
}

/*
 * Handles the color sensor automation setup.
 * Stage 1: Sample current color.
 * Stage 2: Keep moving motors until conditions are met.
 *          a. Currently sensing sampled color.
 *          b. Ensured you moved away from the initial strip.
 *          c. Special case: mode switched, exit immediately.
 * Stage 3: When found, stop moving motors after delay.
 * @param ctl pointer to controller
 */
void colorAutomation(ControllerPtr ctl) {
    if (!sampled) { // stage 1
        sampleColor = recordColorRecordedCheck(); // saves red, green, blue, or none
        currentColor = sampleColor;
        checkInitial = false; // value to check if we moved off the color
        colorFound = false; // value to check if we found the color again
        sampled = true;
    }

    if (!(checkInitial && colorFound) && currentMode == COLOR_AUTOMATION) { // stage 2
        setMode(ctl); // allows exit within loop
        currentColor = recordColorRecordedCheck(); // get current Color (r, g, b, none)
        colorFound = false; // set equal to false to reiterate

        if (currentColor != sampleColor) { // if we moved off the sampled color
            checkInitial = true;
        }

        if (currentColor == sampleColor) { // if we found the color again
            colorFound = true;
        }

        moveMotorsHelper(TOP_MOTOR_SPEED, 0, TOP_MOTOR_SPEED, 0); // move forward
    } else { // stage 3
        moveMotorsHelper(0, 0, 0, 0); // stop robot
        currentMode = MANUAL;
        resetColorVariables();
    }
}

/*
 * Prints color values.
 */
void colorDebug() {
    Console.printf("sampleColor: %d currentColor: %d checkInitial: %d colorFound: %d sampled: %d  ",
         sampleColor, currentColor, checkInitial, colorFound, sampled);
    Console.printf("R: %3d G: %3d B: %3d A: %3d\n", colorArray[RED], colorArray[GREEN], 
        colorArray[BLUE], colorArray[ALPHA]);
}

/*
 * Updates colorArray with new values.
 */
void updateColor() {
    while (!apds.colorAvailable()) { // Wait until color is read from the sensor
        delay(5); 
    }  

    apds.readColor(colorArray[RED], colorArray[GREEN], colorArray[BLUE], colorArray[ALPHA]);
    colorArray[RED] = (int)(colorArray[RED] * redAdjust);
    colorArray[GREEN] = (int)(colorArray[GREEN] * greenAdjust);
}

/*
 * Helper to find specific color.
 * Calculates color using black threshold and checks whether all colors surprass a certain value.
 * @return number associated with color
 */
int recordColorOverCheck() {
    int sum = 0;
    int8_t maxNumber = NONE;
    int8_t maxIndex = NONE;
    uint8_t over = 0;
    uint8_t blackSumThreshold = 20;

    for (int i = 0; i < NUM_COLORS - 1; i++) { // find color
        sum += colorArray[i];

        if (colorArray[i] > maxNumber) {
            maxNumber = colorArray[i];
            maxIndex = i;
        }
        if (colorArray[i] > 32) {
            over++;
        }
    }

    if (sum < blackSumThreshold) { // check black
        return NONE;
    }

    if (over == 3) {
        return NONE;
    }

    return maxIndex;
}

/*
 * Helper to find specific color
 * Calculates color using multipliers.
 * @return number associated with color
 */
int recordColorMultiplierCheck() {
    uint8_t multiplier = 1; // number other colors have to stay below
    int8_t color = NONE; // initialize to none until it finds a color

    if ((colorArray[RED] > colorArray[GREEN] * multiplier) && 
    (colorArray[RED] > colorArray[BLUE] * multiplier)) {
        color = RED;
    } else if ((colorArray[GREEN] > colorArray[RED] * multiplier) && 
    (colorArray[GREEN] > colorArray[BLUE] * multiplier)) {
        color = GREEN;
    } else if ((colorArray[BLUE] > colorArray[RED] * multiplier) && 
    (colorArray[BLUE] > colorArray[GREEN] * multiplier)) {
        color = BLUE;
    }

    return color;
}

/*
 * Helper to find specific color.
 * Calculates color using black and white threshold.
 * @return number associated with color
 */
int recordColorThresholdCheck() {
    uint8_t sum = 0;
    int8_t maxNumber = NONE;
    int8_t maxIndex = NONE;
    uint16_t blackSumThreshold = 40;
    uint16_t whiteSumThreshold = 110;

    for (uint8_t i = 0; i < NUM_COLORS - 1; i++) {
        sum += colorArray[i];

        if (colorArray[i] > maxNumber) {
            maxNumber = colorArray[i];
            maxIndex = i;
        }
    }

    if (sum < blackSumThreshold || sum > whiteSumThreshold) { // check black
        return NONE;
    }

    if (maxIndex == 2 && colorArray[ALPHA] > 80) {
        return NONE;
    }

    return maxIndex;
}



/*
 * Helper to find specific color.
 * Calculates color using predetermined color values and +- threshold.
 * @return number associated with color
 */
int recordColorRecordedCheck() {
    uint8_t rr = 51; // red val on red
    uint8_t rg = 18; // green val on red
    uint8_t rb = 22; // blue val on red
    uint8_t ra = 66;

    uint8_t gr = 54; // red val on green
    uint8_t gg = 65; // green val on green
    uint8_t gb = 36; // blue val on green
    uint8_t ga = 124;

    uint8_t br = 24; // red val on blue
    uint8_t bg = 48; // green val on blue
    uint8_t bb = 57; // blue val on blue
    uint8_t ba = 109;

    uint8_t red = colorArray[RED];
    uint8_t green = colorArray[GREEN];
    uint8_t blue = colorArray[BLUE];
    uint8_t alpha = colorArray[ALPHA];

    if (((red < rr + rangeThreshold) && (red > rr - rangeThreshold)) && 
    ((green < rg + rangeThreshold) && (green > rg - rangeThreshold)) && 
    ((blue < rb + rangeThreshold) && (blue > rb - rangeThreshold)) && 
    ((alpha < ra + rangeThreshold) && (alpha > ra - rangeThreshold))) {
        updateInARow(redInARow + 1, 0, 0);

        if (redInARow > inARowThreshold || !sampled) {
            return RED;
        }
    } else if (((red < gr + rangeThreshold) && (red > gr - rangeThreshold)) && 
    ((green < gg + rangeThreshold) && (green > gg - rangeThreshold)) && 
    ((blue < gb + rangeThreshold) && (blue > gb - rangeThreshold)) && 
    ((alpha < ga + rangeThreshold) && (alpha > ga - rangeThreshold))) {
        updateInARow(0, greenInARow + 1, 0);

        if (greenInARow > inARowThreshold || !sampled) {
            return GREEN;
        }
    } else if (((red < br + rangeThreshold) && (red > br - rangeThreshold)) && 
    ((green < bg + rangeThreshold) && (green > bg - rangeThreshold)) && 
    ((blue < bb + rangeThreshold) && (blue > bb - rangeThreshold)) && 
    ((alpha < ba + rangeThreshold) && (alpha > ba - rangeThreshold))) {
        updateInARow(0, 0, blueInARow + 1);

        if (blueInARow > inARowThreshold || !sampled) {
            return BLUE;
        }
    } else {
        updateInARow(0, 0, 0);
    }

    return NONE;
}

/*
 * Stores how many times a cedrtain color has been read in a row. Used for verification.
 */
void updateInARow(int val1, int val2, int val3) {
    redInARow = val1;
    greenInARow = val2;
    blueInARow = val3;
}

/*
 * Resets color variables for next color automation call.
 */
void resetColorVariables() {
    sampleColor = NONE;
    currentColor = NONE;
    checkInitial = false;
    colorFound = false;
    sampled = false;
    redInARow = 0;
    greenInARow = 0;
    blueInARow = 0;
}

//------------------------------------------------------------------------------------------------//
//--------------------------------------<< MAZE AUTOMATION >>-------------------------------------//
//------------------------------------------------------------------------------------------------//

ESP32SharpIR frontIRSensor(ESP32SharpIR::GP2Y0A21YK0F, FRONT_IR_PIN);
ESP32SharpIR rightIRSensor(ESP32SharpIR::GP2Y0A21YK0F, RIGHT_IR_PIN);

const uint8_t NUM_IR_SENSORS = 2;
float irArray[NUM_IR_SENSORS];

const uint32_t TIME_2880_DEGREES = 14000;
const uint32_t TIME_90_DEGREES = TIME_2880_DEGREES / 32;
const uint32_t TIME_1_DEGREE = TIME_2880_DEGREES / 2880;

uint8_t wallThreshold = 18; // how far until it's considered an open route
uint8_t leftThreshold = 20; // how far left sensor has to be to consider open route
uint8_t rightThreshold = 20; // how far right sensor has to be to consider open route
uint8_t wallAutomationSpeed = TOP_MOTOR_SPEED - 50;

bool rotated = false;
uint32_t lastTurnTime = 0;
bool lastTurnTimeSet = false;
int waitTime = 500;

float DISTANCE_IN_TIME_2880_DEGREES = 560; // CHANGE!!!!
float DISTANCE_IN_TIME_90_DEGREES = DISTANCE_IN_TIME_2880_DEGREES / 32;
float prev = NONE;

void rotate90CWFR(uint32_t addedDelayMicroseconds = 0, bool direction = true);
void rotate90CCWFR(uint32_t addedDelayMicroseconds = 0, bool direction = true);
void rotate90CWLR(uint32_t addedDelayMicroseconds = 0, bool direction = true);
void rotate90CCWLR(uint32_t addedDelayMicroseconds = 0, bool direction = true);
void updateIR();

/*
 * Sets up the wall sensor.
 */
void irSetup() {
    frontIRSensor.setFilterRate(1.0f);
    rightIRSensor.setFilterRate(1.0f);
}

/*
 * Handles the wall sensor automation.
 * This tries to keep the right sensor within the right threshold line. 
 * It doesn't matter too much since when there is a path on the right, it will be very clear.
 * 
 */

 /*
 * Automation that doesn't adjust at all.
 */
void wallAutomationNoCorrectionFR() {
    float frontDistance = irArray[0];
    float rightDistance = irArray[1];

    if (frontDistance > wallThreshold) { // no wall in front -> move forward
        moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0);
    } else { // wall in front -> turn a direction
        bool rotate90cw = (rightDistance > rightThreshold);

        if (rotate90cw) { // if L on wall but R not -> opening on right -> rotate 90 degrees cw
            rotate90CWFR();
        } else { // if L not on wall but R on -> opening on left -> rotate 90 degrees ccw
            rotate90CCWFR();
        }
    }
}



/*
 * Automation that slightly adjuts based off thresholds.
 * Want to stay past wallThreshold.
 */
void wallAutomationMinorCorrectionFR() {
    float frontDistance = irArray[0];
    float rightDistance = irArray[1];
    uint8_t speedAdjust = 0; // CHANGE AS NEEDED
    uint8_t minSweetSpot = 8;
    uint8_t maxSweetSpot = 15;

    if (frontDistance > wallThreshold || rotated) { // no wall in front -> move forward

        if (rightDistance < minSweetSpot) {
            // left slower
            moveMotorsHelper(wallAutomationSpeed - speedAdjust, 0, wallAutomationSpeed, 0);
        } else if (rightDistance > maxSweetSpot) {
            // right slower
            moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed - speedAdjust, 0);
        } else {
            moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0);
        }

        if (!lastTurnTimeSet) {
            lastTurnTime = millis();
            lastTurnTimeSet = true;
        }

        // wait a certain number of time before setting rotate to false again
        if (millis() - lastTurnTime > waitTime) {
            rotated = false;
        }
    } else { // wall in front -> turn a direction
        bool rotate90cw = (rightDistance > rightThreshold);

        if (rotate90cw) { // if L on wall but R not -> opening on right -> rotate 90 degrees cw
            rotate90CWFR();
        } else { // if L not on wall but R on -> opening on left -> rotate 90 degrees ccw
            rotate90CCWFR();
        }

        rotated = true;
        lastTurnTimeSet = false; // reset for next time
    }
}


/*
 * Automation that adjusts based off angle from previous to current within the TIME_90_DEGREES.
 */
void wallAutomationAngleCorrectionFR() { // basically calls updateIR() twice find a fix
    float frontDistance = irArray[0];
    float rightDistance = irArray[1];
    float curr = rightDistance;

    if (prev == NONE) prev = curr; // initialize prev during first run

    float expression = constrain((curr - prev) / DISTANCE_IN_TIME_90_DEGREES, -1.0, 1.0);
    float angle = abs(asin(expression)) * (180.0 / PI); // calculate angle in degrees no negatives
    uint32_t start = millis();

    while ((millis() - start < TIME_90_DEGREES)) {
        updateIR();
        frontDistance = irArray[0];
        rightDistance = irArray[1];

        if (frontDistance > wallThreshold) { // no wall in front -> move forward
            moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0);
            delay(1);
        } else { // wall in front -> turn a direction
            bool rotate90cw = (rightDistance > rightThreshold); // open on right
            bool direction = (curr - prev) > 0 ? true : false; // determine direction of angle
            uint32_t time = TIME_1_DEGREE * angle; // calculate time to turn based off angle

            if (rotate90cw) { // if L on wall but R not -> opening on right -> rotate 90 degrees cw
                rotate90CWFR(time, direction);
            } else { // if L not on wall but R on -> opening on left -> rotate 90 degrees ccw
                rotate90CCWFR(time, direction);
            }

            break; // exit while loop after turn
        }
    }

    //Console.printf("curr: %5.2f prev: %5.2f angle: %5.2f ------- ", curr, prev, angle);
    prev = curr; // after delay, set previous to current. Next iteration will update current.
}

/*
 * Automation that doesn't adjust at all.
 * Despite using the left and right sensors, it will only use the right sensor to adjust to middle.
 * Three cases:
 * 1. L and R on wall -> move forward.
 * 2. L on wall but R not -> rotate 90 degrees clockwise.
 * 3. L not on wall but R on -> rotate 90 degrees counter clockwise.
 */
void wallAutomationNoCorrectionLR() {
    float leftDistance = irArray[0];
    float rightDistance = irArray[1];
    bool leftWall = (leftDistance < leftThreshold);
    bool rightWall = (rightDistance < rightThreshold);

    if (leftWall && rightWall) {
        moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0); // move forward
    } else if (leftWall && !rightWall) { // opening on right
        rotate90CWLR();
    } else if (!leftWall && rightWall) { // opening on left
        rotate90CCWLR();
    }
}

/*
 * Automation that slightly adjuts based off thresholds.
 * Want to stay past wallThreshold.
 */
void wallAutomationMinorCorrectionLR() {
    float leftDistance = irArray[0];
    float rightDistance = irArray[1];
    uint8_t speedAdjust = 20; // CHANGE AS NEEDED
    uint8_t minSweetSpot = 8;
    uint8_t maxSweetSpot = 12;
    bool leftWall = (leftDistance < leftThreshold);
    bool rightWall = (rightDistance < rightThreshold);

    if (leftWall && rightWall) {
        if (rightDistance < minSweetSpot) {
            // left slower
            moveMotorsHelper(wallAutomationSpeed - speedAdjust, 0, wallAutomationSpeed, 0);
        } else if (rightDistance > maxSweetSpot) {
            // right slower
            moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed - speedAdjust, 0);
        } else {
            moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0); // move forward
        }
    } else if (leftWall && !rightWall) { // opening on right
        rotate90CWLR();
    } else if (!leftWall && rightWall) { // opening on left
        rotate90CCWLR();
    }
}

/*
 * Automation that adjusts based off angle from previous to current within the TIME_90_DEGREES.
 */
void wallAutomationAngleCorrectionLR() {
    float leftDistance = irArray[0];
    float rightDistance = irArray[1];
    bool leftWall = (leftDistance < leftThreshold);
    bool rightWall = (rightDistance < rightThreshold);
    float curr = rightDistance;

    if (prev == NONE) prev = curr; // initialize prev during first run

    float expression = constrain((curr - prev) / DISTANCE_IN_TIME_90_DEGREES, -1.0, 1.0);
    float angle = abs(asin(expression)) * (180.0 / PI); // calculate angle in degrees no negatives
    uint32_t start = millis();

    while ((millis() - start < TIME_90_DEGREES )) {
        updateIR();
        leftDistance = irArray[0];
        rightDistance = irArray[1];

        if (leftWall && rightWall) { // no wall in front -> move forward
            moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0);
            delay(1);
        } else { // wall in front -> turn a direction
            int direction = (curr - prev) > 0 ? true : false; // determine direction of angle
            uint32_t time = TIME_1_DEGREE * angle; // calculate time to turn based off angle

            if (leftWall && !rightWall) { // if L on wall but R not -> opening on right -> rotate 90
                                            // degrees clockwise switch to just !rightWall if needed
                rotate90CWLR(time, direction);
            } else if (!leftWall && rightWall) { // if L not on wall but R on -> opening on left -> 
                                                              // rotate 90 degrees counter clockwise
                rotate90CCWLR(time, direction);
            } else { // finished maze or error, just move forward
                moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0);
                delay(1);
            }

            break; // exit while loop after turn
        }
    }

    prev = curr; // after delay, set previous to current. Next iteration will update current.
}

void wallDistanceTest() {
    if (irArray[0] < wallThreshold) {
        moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0);
        delay(TIME_2880_DEGREES);
    }

    moveMotorsHelper(0, 0, 0, 0);

}

 /*
 * Automation that doesn't adjust at all.
 * Want to stay within [8, 12] cm from wall
 */
void wallAutomationTestNoDelay() {
    float frontDistance = irArray[0];
    float rightDistance = irArray[1];

    if (frontDistance <= wallThreshold) { // no wall in front -> move forward
        bool rotate90cw = (rightDistance > rightThreshold);

        if (rotate90cw) { // if L on wall but R not -> opening on right -> rotate 90 degrees cw
            rotate90CWFR();
        } else { // if L not on wall but R on -> opening on left -> rotate 90 degrees ccw
            rotate90CCWFR();
        }
    } else {
        moveMotorsHelper(0, 0, 0, 0); // stop
    }
}

/*
 * Automation that slightly adjuts based off thresholds.
 * Want to stay past wallThreshold.
 */
void wallAutomationTestDelay() {
    float frontDistance = irArray[0];
    float rightDistance = irArray[1];

    if (frontDistance > wallThreshold || rotated) { // no wall in front -> move forward
        moveMotorsHelper(0, 0, 0, 0);

        if (!lastTurnTimeSet) {
            lastTurnTime = millis();
            lastTurnTimeSet = true;
        }

        // wait a certain number of time before setting rotate to false again
        if (millis() - lastTurnTime > waitTime) {
            rotated = false;
        }
    } else { // wall in front -> turn a direction
        bool rotate90cw = (rightDistance > rightThreshold);

        if (rotate90cw) { // if L on wall but R not -> opening on right -> rotate 90 degrees cw
            rotate90CWFR();
        } else { // if L not on wall but R on -> opening on left -> rotate 90 degrees ccw
            rotate90CCWFR();
        }

        moveMotorsHelper(0, 0, 0, 0);
        rotated = true;
        lastTurnTimeSet = false; // reset for next time
    }
}

/*
 * Prints the wall sensor values.
 */
void wallDebugSimple() {
    Console.printf("frontIR: %f rightIR: %f\n", irArray[0], irArray[1]);
}

    /*
 * Automation that adjusts based off angle from previous to current within the TIME_90_DEGREES.
 */
void wallDebugAdvanced() { // basically calls updateIR() twice find a fix
    float curr = irArray[0];
    float prev = irArray[1]; // initialize prev during first run
    float expression = constrain((curr - prev) / DISTANCE_IN_TIME_90_DEGREES, -1.0, 1.0);
    float angle = abs(asin(expression)) * (180.0 / PI);

    if (curr > wallThreshold) { // no wall in front -> move forward
        //moveMotorsHelper(wallAutomationSpeed, 0, wallAutomationSpeed, 0);
    } else { // wall in front -> turn a direction
        bool rotate90cw = (prev > rightThreshold); // open on right

        if (rotate90cw) { // if L on wall but R not -> opening on right -> rotate 90 degrees cw
            //rotate90CWFR(time, direction);
        } else { // if L not on wall but R on -> opening on left -> rotate 90 degrees ccw
             //rotate90CCWFR(time, direction);
        }
    }

    Console.printf("curr: %5.2f prev: %5.2f angle: %5.2f ------- \n", curr, prev, angle);
}

/*
 * Updates irArray with new values.
 * 6-80 cm
 */
void updateIR() {
    irArray[0] = frontIRSensor.getDistanceFloat();
    irArray[1] = rightIRSensor.getDistanceFloat();
}

/*
 * Rotates robot 90 degrees clockwise.
 */
void rotate90CWFR(uint32_t addedDelay, bool direction) {
    moveMotorsHelper(TOP_MOTOR_SPEED, 0, 0, TOP_MOTOR_SPEED);

    if (direction) {
        delay(TIME_90_DEGREES + addedDelay);
    } else {
        delay(TIME_90_DEGREES - addedDelay);
    }
}

/*
 * Rotates robot 90 degrees counter clockwise.
 */
void rotate90CCWFR(uint32_t addedDelay, bool direction) {
    moveMotorsHelper(0, TOP_MOTOR_SPEED, TOP_MOTOR_SPEED, 0);

    if (direction) {
        delay(TIME_90_DEGREES - addedDelay);
    } else {
        delay(TIME_90_DEGREES + addedDelay);
    }
}

/*
 * Rotates robot 90 degrees clockwise.
 */
void rotate90CWLR(uint32_t addedDelay, bool direction) {
    moveMotorsHelper(TOP_MOTOR_SPEED, 0, 0, 0);

    if (direction) {
        delay((TIME_90_DEGREES * 2) + addedDelay);
    } else {
        delay((TIME_90_DEGREES * 2) - addedDelay);
    }
}

/*
 * Rotates robot 90 degrees counter clockwise.
 */
void rotate90CCWLR(uint32_t addedDelay, bool direction) {
    moveMotorsHelper(0, 0, TOP_MOTOR_SPEED, 0);

    if (direction) {
        delay((TIME_90_DEGREES * 2) - addedDelay);
    } else {
        delay((TIME_90_DEGREES * 2) + addedDelay);
    }
}

//------------------------------------------------------------------------------------------------//
//-------------------------------------------<< SETUP >>------------------------------------------//
//------------------------------------------------------------------------------------------------//

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);
    colorSetup();
    Serial.begin(115200);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    motorSetup();
    launchMotorSetup();
    lineSetup();
    irSetup();
    angleServoSetup();
}

//------------------------------------------------------------------------------------------------//
//--------------------------------------------<< LOOP >>------------------------------------------//
//------------------------------------------------------------------------------------------------//

void loop() {
    vTaskDelay(1); // Ensures WDT does not get triggered when no controller is connected
    BP32.update();
    bool debug = false;
    bool automate = true;
    for (auto myController : myControllers) { // Only execute code when controller is connected
        if (myController && myController->isConnected() && myController->hasData()) {

            setMode(myController);
            cleanTerminalSimple();
            digitalWrite(ONBOARD_LED_PIN, HIGH);

            if (debug) Console.printf("Current mode: %s ------- ", MODES[currentMode]);

            switch (currentMode) {
                case MANUAL:
                    if (sampled) resetColorVariables();

                    if (automate) {
                        checkLaunchMotor(myController);
                        moveMotorsSimple(myController);
                        moveAngleServo(myController);
                    }

                    if (debug) dumpGamepad(myController);

                    break;
                case COLOR_AUTOMATION:
                    updateColor();

                    if (automate) colorAutomation(myController);

                    if (debug) colorDebug();

                    break;
                case WALL_AUTOMATION:
                    updateIR();

                    if (automate) wallAutomationNoCorrectionFR();

                    if (debug) wallDebugSimple();

                    break;
                case LINE_AUTOMATION:
                    if (!lineCalibrated) startCalibration();

                    updateLine();

                    if (automate) lineAutomationB();

                    if (debug) lineDebug();
                    
                    break;
            }
            digitalWrite(ONBOARD_LED_PIN, LOW);
        }
    }
}