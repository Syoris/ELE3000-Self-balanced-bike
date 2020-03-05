#include <Arduino.h>

#include <IMU.h>
#include <Motor_Controller.h>
#include <IR_Receiver.h>

#include "Servo.h"

// Define
#define LED_PIN 13


bool blinkState = false;

//Servo pin
Servo servo;
double servoAngle;

bool imuReady = false;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    while(!Serial.available());
    // Attach servo
    //servo.attach(7);
    
    // Setup IMU
    //imuReady = IMU_Setup();

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}

double angle = 0;


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // // If IMU not ready failed, don't try to do anything
    // if (!imuReady) return;

    // IMU_Compute(ypr);

    // if(ypr[0] != -1 && ypr[1] != -1 && ypr[2] != -1){
    //     servoAngle = ypr[2] * 180/M_PI+90;
    //     servo.write(servoAngle);
    // } 

    // angle = flywheelMotor.readAngle();
    // Serial.println(angle);

    // double error = 720 - angle;
    // double Kp = 1.8;

    // if(error > 0 )
    //     flywheelMotor.setMotorSpeed(error*Kp, CW);
    // else
    //     flywheelMotor.setMotorSpeed(error*Kp, CCW);

    flywheelMotor.setMotorSpeed(500, CW);
    Serial.println(flywheelMotor.getSpeed());

}

