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
    //! Initialize serial communication
    Serial.begin(9600);

    //! configure LED for output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    while(!Serial.available()); //Wait for serial command

    //! Attach servo
    //servo.attach(7);
    
    //! Setup IMU
    imuReady = IMU_Setup();

    //! Initialize motor and timer
    flywheelMotor.initMotor();
}

double motorAngle = 0;
double angle = 0;
int max_speed = 255;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    //! Test du IMU
    // // If IMU not ready failed, don't try to do anything
    // if (!imuReady) return;

    // IMU_Compute(ypr);
    // angle = ypr[1];
    // Serial.println(angle);
    
    // int speed = (angle > 0 ? max_speed: -max_speed);
    // flywheelMotor.setMotorSpeedPID(speed);


    //! Test de l'angle
    // angle = flywheelMotor.readAngle();
    // double error = 360 - angle;
    // double Kp = 1.4;
    // if(error > 0 )
    //     flywheelMotor.setMotorSpeed(error*Kp, CW);
    // else
    //     flywheelMotor.setMotorSpeed(error*Kp, CCW);

    // Serial.println(angle);


    //! Test de la vitesse
    for(int i = -256; i<256; i+=5){
        flywheelMotor.setMotorSpeedPID(i);


        Serial.print("Voltage: ");
        Serial.print(i);
        Serial.print("\t Speed (deg/sec): ");
        Serial.print(flywheelMotor.getSpeed());
        Serial.print("\t Speed (rpm): ");
        Serial.println(flywheelMotor.getSpeedRPM());
        delay(300);
    }
}

