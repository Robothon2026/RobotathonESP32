// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"
#include <QTRSensors.h> // Line sensor library
#include <ESP32SharpIR.h> // IR sensor library
#include <Wire.h>
#include <Arduino_APDS9960.h> // Color sensor library
#include <bits/stdc++.h>

// LED pin
#define ONBOARD_LED_PIN 2

// Right motor pins
#define IN1 19
#define IN2 18

// Left motor pins
#define IN3 17
#define IN4 16

// Line sensor pins       0   1   2   3   4   5   6   7
#define LINE_FOLLOW_PINS {34, 35, 32, 33, 25, 26, 27, 14}

// IR sensor pins
#define IR_PIN 13

// Color sensor pins and settings
#define APDS9960_INT_PIN 4
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

const int TOP_MOTOR_SPEED = 255; // Max speed of motors
const int MAX_JOYSTICK_INPUT = 512; // Max speed value from controller
const int NUM_LINE_SENSORS = 8; // Number of line sensors
const int I2C_FREQUENCY = 100000; // I2C frequency for color sensor

// Controller
extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Line
QTRSensors qtr;

// IR
ESP32SharpIR irSensor(ESP32SharpIR::GP2Y0A21YK0F, IR_PIN);

// Color
TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT_PIN);

int currentMode = 0; // Current mode for robot

/*
 * This method clears the terminal screen by printing 10 new lines.
 */
void cleanTerminal() {
    for (int i = 0; i < 10; i++) {
        Console.println();
    }
}

/*
 * This method calibrates the line sensor by taking multiple readings.
 */
void calibrateLineSensors() {
    for (uint8_t i = 0; i < 250; i++) {
        cleanTerminal();
        Console.printf("Calibrating %d/250\n", i);
        qtr.calibrate();
        delay(20);
    }
}

/*
 * This method sets the current mode based on the controller input.
 * @param ctl pointer to controller
 */
void setMode(ControllerPtr ctl) {
    if (ctl->dpad()) {
        currentMode = 0; // Manual mode
    } else if (ctl->b()) {
        currentMode = 1; // Color mode
    } else if (ctl->a()) {
        currentMode = 2; // Wall mode
    } else if (ctl->x()) {
        currentMode = 3; // Mechanical mode
    } else if (ctl->y()) {
        currentMode = 4; // Line follow mode
    }
}

/*
 * This method prints all button values for the controller.
 * @param ctl pointer to controller
 */
void dumpGamepad(ControllerPtr ctl) {
    cleanTerminal();
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
 * This method sets up the motor pins as outputs.
 */
void motorPinsSetup() {
    // Set motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT); // Are these needed? Try removing and if it works, remove.
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

/*
 * This method is called every time the loop function runs and a controller is connected.
 * It handles the movement of the robot based on the controller input.
 * Clockwise-Counterclockwise rotation is controlled by the left-right bumpers.
 * @param crt pointer to controller
 */
void moveMotors(ControllerPtr crt) {
    int lY = crt->axisY(); // Left joystick Y axis
    int aLY = abs(lY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // Adjusted Left joystick Y axis for speed input [0, TOP_MOTOR_SPEED]
    int rY = crt->axisRY();// Right joystick Y axis
    int aRY = abs(rY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // Adjusted Right joystick Y axis for speed input [0, TOP_MOTOR_SPEED]

    bool lForward = (lY <= 0); // Determine if left joystick is moving forward or backward
    bool rForward = (rY <= 0); // Determine if right joystick is moving forward or backward

    if (lForward) { // if left joystick is forward, move motor forward at adjusted speed
        analogWrite(IN1, aLY);
        analogWrite(IN2, 0);
    } else { // else, move motor backwards at adjusted speed
        analogWrite(IN1, 0);
        analogWrite(IN2, aLY);
    }

    if (rForward) { // if right joystick is forward, move motor forward at adjusted speed
        analogWrite(IN3, aRY);
        analogWrite(IN4, 0);
    } else { // else, move motor backwards at adjusted speed
        analogWrite(IN3, 0);
        analogWrite(IN4, aRY);
    }
}

/*
 * It handles the movement of the servo based on the controller input.
 * @param crt pointer to controller
 */
void moveServo(ControllerPtr crt) {
    // Not implemented yet.
}

/*
 * This method sets up the color sensor and I2C communication.
 */
void colorAutomationSetup() {
    //sets up I2C protocol and color sensor
    I2C_0.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
    apds.setInterruptPin(APDS9960_INT_PIN);
    apds.begin();
}

/*
 * This method prints the color values for debugging purposes.
 */
void colorAutomationDebug(int colors[]) {
    cleanTerminal();
    Console.printf("R: %3d G: %3d B: %3d A: %3d\n", colors[0], colors[1], colors[2], colors[3]);
    delay(100);
}

/*
 * This method handles the color sensor automation setup.
 */
void colorAutomation() {
    int colors[4]; // Array to hold RGBA values
    while (!apds.colorAvailable()) {
        delay(5);
    }
    apds.readColor(colors[0], colors[1], colors[2], colors[3]); // Updates array
    colorAutomationDebug(colors);
}

/*
 * This method sets up the wall sensor.
 */
void wallAutomationSetup() {
    irSensor.setFilterRate(1.0f); // Set filter rate to 1.0f (no filtering)
}

/*
 * This method prints the wall sensor values for debugging purposes.
 */
void wallAutomationDebug() {
    cleanTerminal();
    float distance = irSensor.getDistanceFloat();
    Console.printf("Distance: %.2f cm\n", distance);
    delay(100);
}

/*
 * This method handles the wall sensor automation.
 */
void wallAutomation() {
    // Not implemented yet.
    // wallAutomationDebug();
}

/*
 * This method handles the mechanical automation.
 */
void mechanicalAutomation() {
    // Not implemented yet.
}

/*
 * This method sets up the line following sensors.
 */
void lineFollowAutomationSetup() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[])LINE_FOLLOW_PINS, NUM_LINE_SENSORS);
    calibrateLineSensors(); // Calibrate line sensor
}

/*
 * This method prints the line sensor values for debugging purposes.
 * @param sensors array of sensor values
 */
void lineFollowAutomationDebug(uint16_t sensors[]) {
    cleanTerminal();
    Console.printf("Line Sensor Values:\n");
    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
        Console.printf("%d  ", sensors[i]);
    }
    Console.println();
    delay(100);
}

/*
 * This method handles the line following automation.
 */
void lineFollowAutomation() {
    uint16_t sensors[NUM_LINE_SENSORS];
    qtr.readLineBlack(sensors); // Read line sensor values and put into sensors array
    // lineFollowAutomationDebug(sensors);
}

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);

    colorAutomationSetup(); // Setup color sensor

    Serial.begin(115200);

    pinMode(ONBOARD_LED_PIN, OUTPUT); // Setup LED pin
    motorPinsSetup(); // Setup motor pins
    lineFollowAutomationSetup(); // Setup line sensors
    wallAutomationSetup(); // Setup IR sensor
}

void loop() {
    vTaskDelay(1); // Ensures WDT does not get triggered when no controller is connected
    BP32.update(); // Is this needed inside for loop?
    
    // Loop code will only run if controller is connected.
    for (auto myController : myControllers) { // Only execute code when controller is connected
        if (myController && myController->isConnected() && myController->hasData()) {   
            setMode(myController); // Set current mode based on controller input

            // zr shooting sequence: spin launch motor, delay, spin the servo to allow ball in.
            // zl intake sequence: spins motor to intake balls
            // d pad to pivot up and down shooter

            switch (currentMode) {
                case 0: // Manual mode
                    moveMotors(myController);
                    moveServo(myController);
                    break;
                case 1: // Color mode
                    colorAutomation();
                    break;
                case 2: // Wall mode
                    wallAutomation();
                    break;
                case 3: // Mechanical mode
                    mechanicalAutomation();
                    break;
                case 4: // Line follow mode
                    lineFollowAutomation();
                    break;
            }
        }
    }
}