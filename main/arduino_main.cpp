// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"

#define ONBOARD_LED_PIN 2

// Right motor pins
#define IN1 19
#define IN2 18

// Left motor pins 
#define IN3 17
#define IN4 16

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // BP32 library allows for up to 4 concurrent controller connections, but we only need 1

const int topSpeed = 255; // Max speed of motors

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
 * This method is called every time the loop function runs and a controller is connected.
 * It handles the movement of the robot based on the controller input.
 * Clockwise-Counterclockwise rotation is controlled by the left-right bumpers.
 * @param crt pointer to controller
 */
void moveMotors(ControllerPtr crt) {
    int lY = crt->axisY(); // Left joystick Y axis
    int aLY = abs(lY) * 255 / 512; // Adjusted Left joystick Y axis for speed input [0, 255]
    int rY = crt->axisRY();// Right joystick Y axis
    int aRY = abs(rY) * 255 / 512; // Adjusted Right joystick Y axis for speed input [0, 255]

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

void moveServo(ControllerPtr crt) {
    // Not implemented yet.
}

void colorAutomation() {
    // Not implemented yet.
}

void wallAutomation() {
    // Not implemented yet.
}

void mechanicalAutomation() {
    // Not implemented yet.
}

void lineFollowAutomation() {
    // Not implemented yet.
}

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);

    // Set LED pin
    pinMode(ONBOARD_LED_PIN, OUTPUT);

    // Set motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Serial.print(115200);
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