// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"

#define ONBOARD_LED_PIN 2

#define IN1 19
#define IN2 18

#define IN3 17
#define IN4 16

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // BP32 library allows for up to 4 concurrent controller connections, but we only need 1

int topSpeed = 255; // Max speed of motors


void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "DPAD: %d A: %d B: %d X: %d Y: %d LX: %d LY: %d RX: %d RY: %d L1: %d R1: %d L2: %d R2: %d\n",
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
 * This function is called every time the loop function runs and a controller is connected.
 * It handles the movement of the robot based on the controller input.
 * 
 * Controls:
 * Forward-Backward movement is controlled by the left joystick Y axis.
 * Left-Right turning is controlled by the left joystick X axis.
 * Clockwise-Counterclockwise rotation is controlled by the left-right bumpers.
 * 
 * @param crt pointer to controller
 */
void movement(ControllerPtr crt){
    int lY = crt->axisY(); // Left joystick Y axis
    int aLY = abs(lY) * 255 / 512; // Adjusted Left joystick Y axis
    int rX = crt->axisRX();// Right joystick X axis
    int aRX = abs(rX) * 255 / 512; // Adjusted Right joystick X axis
    // bool r1 = crt->r1();   // Right bumper
    // bool l1 = crt->l1();   // Left bumper

    int motorAdjustment = max(0, aLY - aRX);

    if (lY <= -30) { // If pushing joystick forward

        if (rX <= -30) { // Change values if stuck drift occurs
            // Turn Left
            // Reduce speed of left motor
            analogWrite(IN1, motorAdjustment);
            analogWrite(IN2, 0);
            analogWrite(IN3, topSpeed);
            analogWrite(IN4, 0);
        } else if (rX > 30) {
            // Turn Right
            // Reduce speed of right motor
            analogWrite(IN1, aLY);
            analogWrite(IN2, 0);
            analogWrite(IN3, motorAdjustment);
            analogWrite(IN4, 0);
        } else {
            // Move Forward
            analogWrite(IN1, aLY);
            analogWrite(IN2, 0);
            analogWrite(IN3, aLY);
            analogWrite(IN4, 0);
        } // add 4th case for complete stop
    } else if (lY > 30) { // If pulling joystick backward
        if (rX <= -30) {
            // Turn Left
            // Reduce speed of left motor
            analogWrite(IN1, 0);
            analogWrite(IN2, motorAdjustment);
            analogWrite(IN3, 0);
            analogWrite(IN4, aLY);
        } else if (rX > 30) {
            // Turn Right
            // Reduce speed of right motor
            analogWrite(IN1, 0);
            analogWrite(IN2, aLY);
            analogWrite(IN3, 0);
            analogWrite(IN4, motorAdjustment);
        } else {
            // Move Backward
            analogWrite(IN1, 0);
            analogWrite(IN2, aLY);
            analogWrite(IN3, 0);
            analogWrite(IN4, aLY);
        }
    }
}

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    Serial.print(115200);
}

void loop() {
    vTaskDelay(1); // Ensures WDT does not get triggered when no controller is connected
             
    BP32.update(); 
    for (auto myController : myControllers) { // Only execute code when controller is connected
        if (myController && myController->isConnected() && myController->hasData()) {        

            movement(myController);
            //  int l2=myController->l2();
            //  int r2=myController->r2();
            //  if(l2==1){
            //     analogWrite(IN1, 255);
            //     analogWrite(IN2, 0);
            //     analogWrite(IN3, 255);
            //     analogWrite(IN4, 0);
                

            //  }else if(r2==1){
            //     analogWrite(IN1, 0);
            //     analogWrite(IN2, 255);
            //     analogWrite(IN3, 0);
            //     analogWrite(IN4, 255);
            //  }


            //dumpGamepad(myController); // Prints the gamepad state, delete or comment if don't need
        }
    }
}
