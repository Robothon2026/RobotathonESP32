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
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESP32Servo.h>

#define ONBOARD_LED_PIN 2 // LED pin
#define IN1 19 // right motor pins
#define IN2 18
#define IN3 17 // left motor pins
#define IN4 16            
#define IN5 4 // launch motor pins
#define IN6 5
const uint8_t LINE_FOLLOW_PINS[] = {36, 35, 34, 14, 13, 39, 33, 32}; // line sensor pins
#define FRONT_IR_PIN 25 // ir pins
#define RIGHT_IR_PIN 26 
#define APDS9960_INT_PIN 0 // color pins
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define ANGLE_SERVO_PIN 27 // angle servo pin
#define COLLECTION_SERVO_PIN 23 // collection servo pin

const uint8_t TOP_MOTOR_SPEED = 255;
const char* const MODES[] = {"Manual", "Color automation", "Wall automation", "Line automation"};
const short MANUAL = 0;
const uint8_t COLOR_AUTOMATION = 1;
const uint8_t WALL_AUTOMATION = 2;
const uint8_t LINE_AUTOMATION = 3;
int currentMode = 0; // current mode for robot
extern ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // controller

//-----------------------------------------------------------------------------------------------//
//-----------------------------------------<< HELPERS >>-----------------------------------------//
//-----------------------------------------------------------------------------------------------//

/*
 * Cleans terminal to debug easier.
 */
void cleanTerminal() {
    for (int i = 0; i < 25; i++) {
        Console.println();
    }
}

/*
 * Alternative way to clean easier.
 * Removes current line.
 */
void cleanTerminalAlt() {
    Console.print('\r');
    Console.print("\x1B[2K");
}

/*
 * Sets cursor back to beginning for next clear call.
 */
void writeTerminalAlt() {
    Console.write('\r');
}

/*
 * Sets current mode based on controller input.
 * @param ctl pointer to controller
 */
void setMode(ControllerPtr ctl) {
    if (ctl->b()) {
        currentMode = MANUAL; // Manual mode
    } else if (ctl->a()) {
        currentMode = COLOR_AUTOMATION; // Color mode
    } else if (ctl->x()) {
        currentMode = WALL_AUTOMATION; // Wall mode
    } else if (ctl->y()) {
        currentMode = LINE_AUTOMATION; // Line mode
    }
}

//-----------------------------------------------------------------------------------------------//
//--------------------------------------<< MOTOR MOVEMENT >>-------------------------------------//
//-----------------------------------------------------------------------------------------------//

const uint16_t MAX_JOYSTICK_INPUT = 512;

void moveMotorsHelper(int leftSpeedForward, int leftSpeedBackward, int rightSpeedForward, int rightSpeedBackward);

/*
 * Sets up the motor pins as outputs.
 */
void motorSetup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
}

/*
 * Handles the movement of the robot.
 * Left joystick controls left motor and right joystick controls right motor.
 * @param ctl pointer to controller
 */
void moveMotors(ControllerPtr ctl) {
    int lY = ctl->axisY(); // Left joystick Y axis
    int aLY = abs(lY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // Adjusted Left joystick Y axis for speed input [0, TOP_MOTOR_SPEED]
    int rY = ctl->axisRY();// Right joystick Y axis
    int aRY = abs(rY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // Adjusted Right joystick Y axis for speed input [0, TOP_MOTOR_SPEED]

    bool lForward = (lY <= -200); // Determine if left joystick is moving forward
    bool lBackward = (lY >= 200); // Determine if left joystick is moving backward
    bool rForward = (rY <= -200);
    bool rBackward = (rY >= 200);

    if (lForward) { // if left joystick is forward, move motor forward at adjusted speed
        analogWrite(IN1, aLY);
        analogWrite(IN2, 0);
    } else if (lBackward) { // else, move motor backwards at adjusted speed
        analogWrite(IN1, 0);
        analogWrite(IN2, aLY);
    } else {
        analogWrite(IN1, 0);
        analogWrite(IN2, 0);
    }

    if (rForward) { // if right joystick is forward, move motor forward at adjusted speed
        analogWrite(IN3, aRY);
        analogWrite(IN4, 0);
    } else if (rBackward) { // else, move motor backwards at adjusted speed
        analogWrite(IN3, 0);
        analogWrite(IN4, aRY);
    } else {
        analogWrite(IN3, 0);
        analogWrite(IN4, 0);
    }
}

/*
 * Handles the movement of the robot differently than main.
 * Left joystick controls both motors and the right joystick turns it to one direction.
 * @param ctl pointer to controller
 */
void altMoveMotors(ControllerPtr ctl) {
    int lY = ctl->axisY(); // left joystick y-value
    int aLY = abs(lY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // adjusted value
    int rX = ctl->axisRX(); // right joystick x-value
    int aRX = abs(rX) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // adjusted value
    bool lT = ctl->l1();
    bool rT = ctl->r1(); // not used most of the time. Ask mentor if this will affect performance

    int motorAdjustment = max(0, aLY - aRX); // adjusts motors for either side
    bool forward = (lY <= -200);
    bool backward = (lY >= 200);
    bool left = (rX <= -30);
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
        if (lT) { // spin counter-clockwise
            moveMotorsHelper(0, TOP_MOTOR_SPEED, TOP_MOTOR_SPEED, 0);
        } else if (rT) { // spin clockwise
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
        "DPAD: %2d A: %2d B: %2d X: %2d Y: %2d LX: %4d LY: %4d RX: %4d RY: %4d L1: %2d R1: %2d L2: %2d R2: %2d\n",
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
void moveMotorsHelper(int leftSpeedForward, int leftSpeedBackward, int rightSpeedForward, int rightSpeedBackward) {
    analogWrite(IN1, leftSpeedForward);
    analogWrite(IN2, leftSpeedBackward);
    analogWrite(IN3, rightSpeedForward);
    analogWrite(IN4, rightSpeedBackward);
}

//-----------------------------------------------------------------------------------------------//
//--------------------------------------<< MOTOR LAUNCHER >>-------------------------------------//
//-----------------------------------------------------------------------------------------------//

bool launchMotors = false;
bool launchMotorsRamped = false;

void rampUpLaunchMotor();
void rampDownLaunchMotor();
void moveLaunchMotorsHelper(int speedForward, int speedBackward);

/* 
 * Checks to see if launch motors need to be ramped up/down or neither.
 * @param ctl pointer to control
 */
void checkLaunchMotors(ControllerPtr ctl) {
    launchMotors = (ctl->r2()) ? true : false;
    if (launchMotors && !launchMotorsRamped) {
        rampUpLaunchMotor();
        launchMotorsRamped = true;
    } else if (!launchMotors && launchMotorsRamped) {
        rampDownLaunchMotor();
        launchMotorsRamped = false;
    }
}

/*
 * Ramp launch motors up so it doens't reach max speed instantly.
 */
void rampUpLaunchMotor() {
    int initialSpeed = 55;
    for (int i = initialSpeed; i <= 200; i++) {
        moveLaunchMotorsHelper(i, 0);
        delay(5);
    }
}

/*
 * Ramp down launch motors so it doesn't stop instantly.
 */
void rampDownLaunchMotor() {
    int initialSpeed = 255;
    for (int i = initialSpeed; i >= 55; i--) {
        moveLaunchMotorsHelper(i, 0);
        delay(5);
    }
    moveLaunchMotorsHelper(0, 0);
}

/*
 * Helper to simplify launch motor control.
 * @param speedForward motor speed forward
 * @param speedBackward motor speed backward
 */
void moveLaunchMotorsHelper(int speedForward, int speedBackward) {
    analogWrite(IN5, speedForward);
    analogWrite(IN6, speedBackward);
}

//-----------------------------------------------------------------------------------------------//
//------------------------------------------<< SERVO >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

Servo angleServo;
Servo collectionServo;

/*
 * Setups up the angle and collection servo.
 */
void servoSetup() {
    angleServo.attach(ANGLE_SERVO_PIN);
    collectionServo.attach(COLLECTION_SERVO_PIN);
}

/*
 * Handles the movement of the angle servo.
 * @param ctl pointer to controller
 */
void moveAngleServo(ControllerPtr ctl) {
    // Not implemented yet.
}

/*
 * Handles the movement of the collection servo.
 * @param ctl pointer to controller
 */
void moveCollectionServo(ControllerPtr ctl) {
    // Not implemented yet.
}

//-----------------------------------------------------------------------------------------------//
//-------------------------------------<< LINE AUTOMATION >>-------------------------------------//
//-----------------------------------------------------------------------------------------------//

const uint8_t NUM_LINE_SENSORS = sizeof(LINE_FOLLOW_PINS) / sizeof(LINE_FOLLOW_PINS[0]);
QTRSensors qtr;// line
uint16_t lineArray[NUM_LINE_SENSORS]; // array holding information for NUM_LINE_SENSORS
int kP= 0.5; // proportional constant for line following
int kD = 0; // derivative constant for line following
int kI = 0; // integral constant for line following
int lastEror = 0; // last error for derivative calculation
int sum = 0; // sum of errors for integral calculation
int previous = 0; // store previous value of distance from wall

void calibrateLineSensors();
int lineHelper();

/*
 * Sets up the line following sensors.
 */
void lineSetup() {
    qtr.setTypeAnalog();
    qtr.setSensorPins(LINE_FOLLOW_PINS, NUM_LINE_SENSORS);
    calibrateLineSensors(); // Calibrate line sensor
}

/*
 * Handles the line following automation.
 */
void lineAutomation() {
    //Fabian's ALGORITHIM
    int speed = TOP_MOTOR_SPEED; // top speed of motors
    int correction = lineHelper(); // The correction for the lineHelper that will fix the robot's path
    moveMotorsHelper(speed + correction ,0 ,speed - correction ,0); // move motors with correction applied to speed
    /* if correction is negative, move left by adjusting the left wheel slower while adjusitng the right wheel faster
    if correction is positive, move right by adjusting the right wheel slower while adjusitng the left wheel faster
    if correction is zero, move straight forward at full speed
    I decided to not use the invert sensors function because it is easers to just use 1,000 for black and 0 for white
    as if i use it the other way i would have to invert everything and also i wouldn't know how that would be able to work becuase
    right now the lineHelper function works by multiplying the sensor values by the weights but if its 1,000 in the middle and 0 on white
    then the error would go based off the middle which i dont think would be as effecient and just more complicated then using 0
    for white and 1,000 for black
    */
}

/*
 * Prints the line sensor values.
 */
void lineDebug() {
    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
        Console.printf("%d  ", lineArray[i]);
    }
    Console.println();
}

/*
 * Updates lineArray with new values.
 */
void updateLine() {
    qtr.readLineBlack(lineArray);
}

int lineHelper() {
    int Error = 0; // used to calculate the error from the line
    int weights[] = {-3, -2, -1, 0, 0, 1, 2, -3}; // Adjust weights based on number of sensors
    for(int i = 0; i < NUM_LINE_SENSORS; i++) { // for loop to go through all sensors
        Error = lineArray[i] * weights[i]; // calculates the weighted error with the value of the sensor                 
    }                                       // example: if leftmost sensor is on black (1000) and all others are white (0)
                                            // then error = 0 * -3 + 1,000 * -2 +  1,000 * -1 + 0 * 0 + 0* 0 + 0 * 1 + 0 * 2 + 0 * 3 = -3000
    Error = Error / 1000; // Normalize error by diving by the highest sensor value that is 1000;
    int P = kP * Error; // equation for the porpotional term for the immediate correction
    int D = kD * (Error - lastEror); // equation for the derivative term for appliying a small brake on the porpotional term
    sum = sum + Error; // sum of all errors up to now for integral term to use to track long term errors over the course
    int I = kI * sum; // equation for the integral term to correct for long term error and apply small amount of correction
                      // The integral term is very small and only affects the robot if a long and difficult course
                      // I think the integral is optional if we want speed but we would have to test  
    lastEror = Error; // Store current error for the next calculation for the derivative term 
    return P + D + I; // returnt the added up corrections into one value
}

/*
 * Calibrates line sensors over 250 iterations. 
 * Increase if need be.
 */
void calibrateLineSensors() {
    for (uint8_t i = 0; i < 250; i++) {
        cleanTerminal();
        Console.printf("Calibrating %d/250\n", i + 1);
        qtr.calibrate();
        delay(20);
    }
    Console.println("Calibration done!");
}

//-----------------------------------------------------------------------------------------------//
//------------------------------------<< COLOR AUTOMATION >>-------------------------------------//
//-----------------------------------------------------------------------------------------------//

const uint8_t NUM_COLORS = 4;
const int8_t NONE = -1;
const uint8_t RED = 0;
const uint8_t GREEN = 1;
const uint8_t BLUE = 2;
const uint8_t ALPHA = 3;
const int I2C_FREQUENCY = 100000; // I2C frequency for color sensor
TwoWire I2C_0 = TwoWire(0);// color
int colorArray[NUM_COLORS]; // array holding information for r, g, b, a
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT_PIN);
int sampleColor = -1; // saves red, green, or blue (no other colors)
int currentColor = -1;
bool checkInitial = false; // value to check if we moved off the color
bool colorFound = false; // value to check if we found the color again
bool sampled = false;

int recordColor();
int recordColorAlt();
void resetColorVariables();

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
 * Stage 1: Sample current color
 * Stage 2: Keep moving motors until conditions are met
 *          a. currently sensing sampled color
 *          b. ensured you moved away from the initial strip
 *          c. special case: mode switched, exit immediately
 * Stage 3: When found, stop moving motors after delay.
 * @param ctl pointer to controller
 */
void colorAutomation(ControllerPtr ctl) { // ask mentor if global variable significantly affects performance
    // Stage 1
    if (!sampled) {
        sampleColor = recordColor(); // saves red, green, blue, or none
        currentColor = sampleColor;
        checkInitial = false; // value to check if we moved off the color
        colorFound = false; // value to check if we found the color again
        sampled = true;
    }
    
    // Stage 2
    if (!(checkInitial && colorFound) && currentMode == COLOR_AUTOMATION) {
        Console.print(currentMode);
        setMode(ctl); // allows exit within loop
        currentColor = recordColor(); // get current Color (r, g, b, none)
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
    Console.printf("sampleColor: %d currentColor: %d checkInitial: %d colorFound: %d sampled: %d  ", sampleColor, currentColor, checkInitial, colorFound, sampled);
    Console.printf("R: %3d G: %3d B: %3d A: %3d\n", colorArray[RED], colorArray[GREEN], colorArray[BLUE], colorArray[ALPHA]);
}

/*
 * Updates colorArray with new values.
 */
void updateColor() {
    while (!apds.colorAvailable()) { delay(5); } // Wait until color is read from the sensor 
    apds.readColor(colorArray[RED], colorArray[GREEN], colorArray[BLUE], colorArray[ALPHA]);
}

/*
 * Helper to find specific color
 * -1: None 0: red 1: green 2: blue
 * @return number associated with color
 */
int recordColor() {
    int sum = 0;
    int maxNumber = -1;
    int maxIndex = -1;
    int over = 0;
    int blackSumThreshold = 200;
    for (int i = 0; i < NUM_COLORS - 1; i++) {
        sum += colorArray[i];
        if (colorArray[i] > maxNumber) {
            maxNumber = colorArray[i];
            maxIndex = i;
        }
        if (colorArray[i] > 220) {
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
 * -1: None 0: red 1: green 2: blue
 * @return number associated with color
 */
int recordColorAlt() {
    int multiplier = 1; // number other colors have to stay below
    int color = NONE; // initialize to none until it finds a color
    if ((colorArray[RED] > colorArray[GREEN] * multiplier) && (colorArray[RED] > colorArray[BLUE] * multiplier)) {
        color = RED;
    } else if ((colorArray[GREEN] > colorArray[RED] * multiplier) && (colorArray[GREEN] > colorArray[BLUE] * multiplier)) {
        color = GREEN;
    } else if ((colorArray[BLUE] > colorArray[RED] * multiplier) && (colorArray[BLUE] > colorArray[GREEN] * multiplier)) {
        color = BLUE;
    }
    return color;
}

/*
 * Helper to reset color variables for next color automation call.
 */
void resetColorVariables() {
    sampleColor = -1;
    currentColor = -1;
    checkInitial = false;
    colorFound = false;
    sampled = false;
}

//-----------------------------------------------------------------------------------------------//
//-------------------------------------<< MAZE AUTOMATION >>-------------------------------------//
//-----------------------------------------------------------------------------------------------//

const uint8_t NUM_IR_SENSORS = 3;
ESP32SharpIR frontIRSensor(ESP32SharpIR::GP2Y0A21YK0F, FRONT_IR_PIN);
ESP32SharpIR rightIRSensor(ESP32SharpIR::GP2Y0A21YK0F, RIGHT_IR_PIN);
float irArray[NUM_IR_SENSORS];
const int TIME_90_DEGREES = 1000; 

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
 */
void wallAutomation() {
    int wallThreshold = 20; // how far until it's considered an open route
    int rightThreshold = 10; // how far you want the robot from the right side
    int speedAdjust = 20; // CHANGE AS NEEDED 
    int multiplier = 2;
    float frontDistance = irArray[1];
    float rightDistance = irArray[2];

    void rotate90CW();
    void rotate90CCW();
    
    //** distance debugging */
    // Console.printf("Front Distance: %f\n", frontDistance);
    // Console.printf("Right Distance: %f\n", rightDistance);

    float current = rightDistance;
    float direction = current - previous; // true: moving away from wall false: moving towards wall
    if (irArray[1] > wallThreshold) { // no wall in front -> move forward
        if (irArray[2] > rightThreshold) { // farther than where needed, turn right
            if (irArray[2] > wallThreshold) { // too far from where needed
                multiplier = 2; // or create manual adjustment where you actually turna nd guarentee its within wall boundary
            }
            moveMotorsHelper(TOP_MOTOR_SPEED, 0, TOP_MOTOR_SPEED - (speedAdjust * multiplier), 0);
        } else { // where you need to be, straighten up
            if (direction > 5) { // moving away, turn right
                moveMotorsHelper(TOP_MOTOR_SPEED, 0, TOP_MOTOR_SPEED - (speedAdjust * multiplier), 0);
            } else if (direction < -5) { // moving towards, turn left
                moveMotorsHelper(TOP_MOTOR_SPEED - (speedAdjust * multiplier), 0, TOP_MOTOR_SPEED, 0);
            } else {
                moveMotorsHelper(TOP_MOTOR_SPEED, 0, TOP_MOTOR_SPEED, 0);
            }
        }
    } else { // wall in front -> turn a direction
        bool rotate90cw = (irArray[2] > wallThreshold);
        if (rotate90cw) { // if L on wall but R not -> opening on right -> rotate 90 degrees clockwise
            rotate90CW();
        } else { // if L not on wall but R on -> opening on left -> rotate 90 degrees counter clockwise
            rotate90CCW();
        }
    }
    previous = current;
}

/*
 * Prints the wall sensor values.
 */
void wallDebug() {
    Console.printf("frontIR: %f rightIR: %f\n", irArray[1], irArray[2]);
}

/*
 * Updates irArray with new values.
 * 10-80 cm
 */
void updateIR() {
    irArray[1] = frontIRSensor.getDistanceFloat();
    irArray[2] = rightIRSensor.getDistanceFloat();
}

/*
 * Rotates robot 90 degrees clockwise.
 */
void rotate90CW() {
    moveMotorsHelper(TOP_MOTOR_SPEED, 0, 0, TOP_MOTOR_SPEED);
    delay(TIME_90_DEGREES);
}

/*
 * Rotates robot 90 degrees counter clockwise.
 */
void rotate90CCW() {
    moveMotorsHelper(0, TOP_MOTOR_SPEED, TOP_MOTOR_SPEED, 0);
    delay(TIME_90_DEGREES);
}

//-----------------------------------------------------------------------------------------------//
//------------------------------------------<< SETUP >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);
    colorSetup(); // Setup color sensor
    Serial.begin(115200);
    pinMode(ONBOARD_LED_PIN, OUTPUT); // Setup LED pin
    motorSetup(); // Setup motor pins
    //lineSetup(); // Setup line sensors
    irSetup(); // Setup IR sensor
}

//-----------------------------------------------------------------------------------------------//
//-------------------------------------------<< LOOP >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

void loop() {
    vTaskDelay(1); // Ensures WDT does not get triggered when no controller is connected
    BP32.update(); // Is this needed inside for loop?
    bool debug = true;
    bool automate = true;
    // Loop code will only run if controller is connected.
    for (auto myController : myControllers) { // Only execute code when controller is connected
        if (myController && myController->isConnected() && myController->hasData()) {
            // wifiConnect(); // Handle WiFi connections
            setMode(myController); // Set current mode based on controller input
                                   // If to inefficient, create loops within each case and handle exiting internally
            // zr shooting sequence: spin launch motor, delay, spin the servo to allow ball in.
            // zl intake sequence: spins motor to intake balls
            // d pad to pivot up and down shooter
            //cleanTerminal();
            Console.printf("Current mode: %s ------- ", MODES[currentMode]);
            switch (currentMode) {
                case MANUAL: // Manual mode
                    if (sampled) resetColorVariables();
                    // altMoveMotors(myController);
                    if (automate) {
                        checkLaunchMotors(myController);
                        moveMotors(myController);
                        moveAngleServo(myController);
                        moveCollectionServo(myController);
                    }
                    if (debug) dumpGamepad(myController);
                    break;
                case COLOR_AUTOMATION: // Color mode
                    updateColor();
                    if (automate) colorAutomation(myController);
                    if (debug) colorDebug();
                    break;
                case WALL_AUTOMATION: // Wall mode
                    updateIR();
                    if (automate) wallAutomation();
                    if (debug) wallDebug();
                    break;
                case LINE_AUTOMATION: // Line follow mode
                    updateLine();
                    if (automate) lineAutomation();
                    if (debug) lineDebug();
                    break;
            }

            if (debug && !automate) delay(100);
        }
    }
}