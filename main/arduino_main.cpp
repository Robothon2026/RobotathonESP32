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
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

#define ONBOARD_LED_PIN 2 // LED pin
#define IN1 19 // Right motor pins
#define IN2 18
#define IN3 17 // Left motor pins
#define IN4 16            
const uint8_t LINE_FOLLOW_PINS[] = {32, 39, 4, 26, 25, 15, 2, 0}; // place holders for line sensor pins
#define IR_PIN 0 // place holders for IR sensor pins
#define APDS9960_INT_PIN 0 // place holders for color sensor pins and settings
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 0

const uint8_t TOP_MOTOR_SPEED = 255; // max speed of motors
const int MAX_JOYSTICK_INPUT = 512; // max speed value from controller
const uint8_t NUM_LINE_SENSORS = sizeof(LINE_FOLLOW_PINS) / sizeof(LINE_FOLLOW_PINS[0]); // number of line sensors
const int I2C_FREQUENCY = 100000; // I2C frequency for color sensor
const char* SSID = "yourSSID"; // WiFi SSID
const char* PASSWORD = "yourPassword"; // WiFi Password
// --- PID CONTROL VARIABLES (Add these to your global section) ---
const int SETPOINT = 0; // The desired position (center of line)
// TUNE THESE: Start with Kd=0 and Ki=0, then tune Kp, then Kd, then Ki.
float Kp = 0.5;   // Proportional Gain have to tune if wrong
float Ki = 0.00;  // Integral Gain have to tune if wrong
float Kd = 10.0;  // Derivative Gain have to tune if wrong

float integral = 0;     // Accumulator for integral term
int lastError = 0;      // Stores error for derivative term

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // controller
QTRSensors qtr;// line
ESP32SharpIR irSensor(ESP32SharpIR::GP2Y0A21YK0F, IR_PIN);// IR
TwoWire I2C_0 = TwoWire(0);// color
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT_PIN);
WiFiServer telnetServer(23); // Telnet server
WiFiClient telnetClient; // Telnet client

int currentMode = 0; // current mode for robot
int colors[4]; // array holding information for r, g, b, a
uint16_t sensors[NUM_LINE_SENSORS]; // array holding information for NUM_LINE_SENSORS

//-----------------------------------------------------------------------------------------------//
//-----------------------------------------<< HELPERS >>-----------------------------------------//
//-----------------------------------------------------------------------------------------------//

/*
 * Reads the 8 line sensors and calculates the continuous position of the line.
 * The position ranges from -3500 (far left) to +3500 (far right) with 0 being the center.
 * This is designed for a black line (value 0) on a white surface (value 1000).
 * @return The calculated continuous line position.
 */
int readLinePosition() {
    // Read the sensors (QTR library handles raw read/calibration)
    // qtr.readLine() typically returns a position, but we need the raw sensors[] array first.
    qtr.read(sensors); 

    // The weights assigned to each sensor: from left (negative) to right (positive)
    // Using a range like -3500 to +3500 is common to ensure a continuous integer range.
    // NOTE: Adjust these weights if your line width/sensor spacing is unusual.
    int weights[NUM_LINE_SENSORS] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500}; 

    long weightedSum = 0;
    long sumOfInvertedReadings = 0;
    
    // Invert the sensor logic for black line (0) on white background (1000).
    //  (close to 1000) is the background (no line).
    //  (close to 0) is the line.
    // We want the line to have a high contribution to the sum, so we use (1000 - sensor[i]).
    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
        // inverted value to correctly weigh the position.
        int invertedReading = 1000 - sensors[i]; 

        sumOfInvertedReadings += invertedReading;
        weightedSum += (long)invertedReading * weights[i];
    }
    
    // If the robot is completely off the line, the sumOfInvertedReadings will be near zero.
    if (sumOfInvertedReadings == 0) {
        // Go staright
        return 0; 
    }
    
    // Calculate the weighted average.
    int position = weightedSum / sumOfInvertedReadings;

    return position; // Returns a value from -3500 (Line on Left) to +3500 (Line on Right)
}

/*
 * Cleans terminal to debug easier.
 */
void cleanTerminal() {
    for (int i = 0; i < 25; i++) {
        Console.println();
    }
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

/*
 * Sets current mode based on controller input.
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
 * Connects to wifi.
 */
void wifiConnect() {
    if (telnetServer.hasClient()) {
        if (!telnetClient || !telnetClient.connected()) {
            if (telnetClient) telnetClient.stop();
            telnetClient = telnetServer.available();
            Serial.println("New Telnet client connected!");
            telnetClient.println("Welcome to ESP32 Debug!");
        } else {
            // Reject additional clients
            telnetServer.available().stop();
        }
    }

    // Example debug info
    if (telnetClient && telnetClient.connected()) {
        telnetClient.printf("Sensor value: %d\n", analogRead(34));
        delay(500);
    }
}

/*
 * Inverts line sensor values to be more intuitive.
 * 0->black
 * 1000->white
 */
void invertSensors() {
    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
        sensors[i] = 1000 - sensors[i]; // Assuming 12-bit ADC, invert the value
    }
}

/*
 * Helper to simplify altMovementMotors.
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
//------------------------------------------<< DEBUG >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

/*
 * Prints all button values for the controller.
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
 * Prints color values.
 */
void colorDebug() {
    cleanTerminal();
    Console.printf("R: %3d G: %3d B: %3d A: %3d\n", colors[0], colors[1], colors[2], colors[3]);
    delay(100);
}

/*
 * Prints the wall sensor values.
 */
void irDebug() {
    cleanTerminal();
    float distance = irSensor.getDistanceFloat();
    Console.printf("Distance: %.2f cm\n", distance);
    delay(100);
}

/*
 * Prints the line sensor values.
 */
void lineDebug() {
    cleanTerminal();
    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
        Console.printf("%d  ", sensors[i]);
    }
    Console.println();
    delay(100);
}

//-----------------------------------------------------------------------------------------------//
//------------------------------------------<< MODES >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

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
    }

    if (rForward) { // if right joystick is forward, move motor forward at adjusted speed
        analogWrite(IN3, aRY);
        analogWrite(IN4, 0);
    } else if (rBackward) { // else, move motor backwards at adjusted speed
        analogWrite(IN3, 0);
        analogWrite(IN4, aRY);
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
    bool lT = ctl->l2();
    bool rT = ctl->r2(); // not used most of the time. Ask mentor if this will affect performance

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
 * Handles the movement of the servo based.
 * @param ctl pointer to controller
 */
void moveServo(ControllerPtr ctl) {
    // Not implemented yet.
}

/*
 * Handles the color sensor automation setup.
 */
void colorAutomation() { // ask mentor if global variable significantly affects performance
    while (!apds.colorAvailable()) {
        delay(5);
    }
    apds.readColor(colors[0], colors[1], colors[2], colors[3]); // Updates array
}

/*
 * Handles the wall sensor automation.
 */
void wallAutomation() {
    // Not implemented yet.
}

/*
 * Handles the mechanical automation.
 */
void mechanicalAutomation() {
    // Not implemented yet.
}

/*
 * Handles the line following automation.
 */
void lineAutomation() {
    int position = readLinePosition(); // Position: -3500 (Left) to +3500 (Right)

    // Error = Setpoint - Position (Error = 0 - Position = -Position)
    int currentError = SETPOINT - position; 

    // 2. CALCULATE PID TERMS
    
    // P-Term: Proportional to current error
    float P = Kp * currentError; 

    // I-Term: Accumulation of error over time
    integral += currentError; 
    float I = Ki * integral;

    // D-Term: Rate of change of the error
    float derivative = currentError - lastError;
    float D = Kd * derivative;

    // 3. CALCULATE TOTAL CORRECTION
    float PID_Correction = P + I + D;
    lastError = currentError; // Store the current error for the next iteration
    
    // The robot will move at a BASE_SPEED (e.g., TOP_MOTOR_SPEED / 2).
    // The PID_Correction is ADDED to one motor and SUBTRACTED from the other.
    
    // The correction is applied to steer the robot back to the center (0).
    // If Error > 0 (Line on Left), Correction > 0.
    //   -> Left Motor Speed is increased to turn Left.
    //   -> Right Motor Speed is decreased to turn Left.
    
    // Base speed is lower value for turning adjustments.
    const int BASE_SPEED = TOP_MOTOR_SPEED / 2; 
    
    int leftMotorSpeed = BASE_SPEED + (int)PID_Correction;
    int rightMotorSpeed = BASE_SPEED - (int)PID_Correction; 

    // Clamp the speeds to prevent exceeding the max PWM value
    //leftMotorSpeed = constrain(leftMotorSpeed, 0, TOP_MOTOR_SPEED);
    //rightMotorSpeed = constrain(rightMotorSpeed, 0, TOP_MOTOR_SPEED);


    moveMotorsHelper(leftMotorSpeed, 0, rightMotorSpeed, 0);
}

//-----------------------------------------------------------------------------------------------//
//------------------------------------------<< SETUP >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

/*
 * Sets up the line following sensors.
 */
void lineSetup() {
    qtr.setTypeAnalog();
    qtr.setSensorPins(LINE_FOLLOW_PINS, NUM_LINE_SENSORS);
    calibrateLineSensors(); // Calibrate line sensor
}

/*
 * Sets up the wall sensor.
 */
void irSetup() {
    irSensor.setFilterRate(1.0f); // Set filter rate to 1.0f (no filtering)
}

/*
 * Sets up the color sensor and I2C communication.
 */
void colorSetup() {
    I2C_0.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
    apds.setInterruptPin(APDS9960_INT_PIN);
    apds.begin();
}

/*
 * Sets up the motor pins as outputs.
 */
void motorSetup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}


void wifiConnectSetup() {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    Serial.println(WiFi.localIP());
    telnetServer.begin();
    telnetServer.setNoDelay(true);
}

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);

    colorSetup(); // Setup color sensor

    Serial.begin(115200);

    pinMode(ONBOARD_LED_PIN, OUTPUT); // Setup LED pin
    motorSetup(); // Setup motor pins
    lineSetup(); // Setup line sensors
    irSetup(); // Setup IR sensor
    //wifiConnectSetup(); // Setup WiFi connection
}

void loop() {
    vTaskDelay(1); // Ensures WDT does not get triggered when no controller is connected
    BP32.update(); // Is this needed inside for loop?
    
    // Loop code will only run if controller is connected.
    for (auto myController : myControllers) { // Only execute code when controller is connected
        if (myController && myController->isConnected() && myController->hasData()) {
            // wifiConnect(); // Handle WiFi connections
            setMode(myController); // Set current mode based on controller input

            // zr shooting sequence: spin launch motor, delay, spin the servo to allow ball in.
            // zl intake sequence: spins motor to intake balls
            // d pad to pivot up and down shooter
            Console.print("Current mode: ");
            Console.println(currentMode);
            switch (currentMode) {
                case 0: // Manual mode
                    // altMoveMotors(myController);
                    moveMotors(myController);
                    moveServo(myController);
                    dumpGamepad(myController);
                    break;
                case 1: // Color mode
                    //colorAutomation();
                    // colorDebug();
                    break;
                case 2: // Wall mode
                    //wallAutomation();
                    // irDebug();
                    break;
                case 3: // Mechanical mode
                    //mechanicalAutomation();
                    break;
                case 4: // Line follow mode
                    lineAutomation();
                    lineDebug();
                    break;
            }
        }
    }
}