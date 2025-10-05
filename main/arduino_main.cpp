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

/*
 * This method clears the terminal screen by printing 10 new lines.
 */
void cleanTerminal() {
    for (int i = 0; i < 10; i++) {
        Console.println();
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
 * This method simplifies the motor movements. 
 * It takes in all motor pins and adjusts accordingly based on forward-backwards movement.
 * @param IN1 left motor forward pin
 * @param IN2 left motor backward pin
 * @param IN3 right motor forward pin
 * @param IN4 right motor backward pin
 * @param leftSpeed speed of left motor
 * @param rightSpeed speed of right motor
 * @param moveForward bool to determine if moving forward (true) or backward (false)
 */
void moveMotorsHelper(int leftSpeed, int rightSpeed, bool moveForward) {
    if (moveForward) {
        analogWrite(IN1, leftSpeed);
        analogWrite(IN2, 0);
        analogWrite(IN3, rightSpeed);
        analogWrite(IN4, 0);
    } else {
        analogWrite(IN1, 0);
        analogWrite(IN2, leftSpeed);
        analogWrite(IN3, 0);
        analogWrite(IN4, rightSpeed);
    }
}

/*
 * This method is called every time the loop function runs and a controller is connected.
 * It handles the movement of the robot based on the controller input.
 * Clockwise-Counterclockwise rotation is controlled by the left-right bumpers.
 * @param crt pointer to controller
 */
void moveMotors(ControllerPtr crt) {
    int lY = crt->axisY(); // Left joystick Y axis
    int aLY = abs(lY) * 255 / 512; // Adjusted Left joystick Y axis
    int rX = crt->axisRX();// Right joystick X axis
    int aRX = abs(rX) * 255 / 512; // Adjusted Right joystick X axis
    
    // Calculate motor adjustment for turning.
    // In case of negative value, it is set to 0
    int motorAdjustment = max(0, aLY - aRX);
    bool moveForward = (lY <= 0); // Determine if moving forward or backward

    // Future: Find value when motor starts to move properly and update all thresholds.
    if (lY <= -30) { // If left joystick is pushed forward
        if (rX <= -30) { // If right joystick is pushed left
            moveMotorsHelper(motorAdjustment, aLY, moveForward);
        } else if (rX > 30) { // If right joystick is pushed right
            moveMotorsHelper(aLY, motorAdjustment, moveForward);
        } else { // If right joystick isn't being pushed either direction -> Just move forward
            moveMotorsHelper(aLY, aLY, moveForward);
        }
    } else if (lY > 30) { // If left joystick is pushed backward
        if (rX <= -30) { // If right joystick is pushed left
            moveMotorsHelper(motorAdjustment, aLY, moveForward);
        } else if (rX > 30) { // If right joystick is pushed right
            moveMotorsHelper(aLY, motorAdjustment, moveForward);
        } else { // If right joystick isn't being pushed either direction -> Just move backward
            moveMotorsHelper(aLY, aLY, moveForward);
        }
    } else { // If left joystick isn't being pushed either direction -> Stop
        moveMotorsHelper(0, 0, moveForward); 
    }
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
            
            // Moves motors based on left and right joystick input.
            moveMotors(myController);

            // Moves servo based on l1 and l2 imputs. Not implemented yet.
            // moveServo(myController);

        }
    }
}