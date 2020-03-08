#include <Arduino.h>

#include <IMU.h>
#include <Motor_Controller.h>
#include <IR_Receiver.h>

#include "Servo.h"

// Define
#define LED_PIN 13

void readSerial();
void IMU_test();


bool imuReady = false;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    //! Initialize serial communication
    Serial.begin(9600);

    //! Configure LED for output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    //! Setup IMU
    imuReady = IMU_Setup();
    flywheelMotor.setTargetSpeed(150);

}

double motorAngle = 0;
double angle = 0;
int max_speed = 255;
bool motorOn = false;
String commande;


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {

    //! Read Serial Input
	readSerial();

}

void IMU_test(){
    //! Test du IMU
    // If IMU not ready failed, don't try to do anything
    if (!imuReady) return;

    IMU_Compute(ypr);
    angle = ypr[1];
    Serial.println(angle);
    
    int speed = (angle > 0 ? max_speed: -max_speed);
    flywheelMotor.setTargetSpeed(speed);
}

void readSerial(){
    while (Serial.available()) {
		char c = Serial.read();  //gets one byte from serial buffer
		commande += c; //makes the string readString
		delay(2);  //slow looping to allow buffer to fill with next character
	}

	if (commande.length() > 0) {
		if (commande[0] == '#') {
			commande = commande.substring(1);
			int len = commande.length();
			commande.remove(len-1, 1);

			Serial.print("Commande: ");
			Serial.println(commande);

            if(commande == "On"){
                motorOn = true;
                flywheelMotor.startMotor();
            }
            else if(commande == "Off"){
                motorOn = false;
                flywheelMotor.stopMotor();
            }

            else if(commande.startsWith("setSpeed")){
                float newSpeed = commande.substring(8).toFloat();              
                Serial.print("New speed: ");
                Serial.println(newSpeed);
                flywheelMotor.setTargetSpeed(newSpeed);
            }

            else if(commande.startsWith("setKp")){
                float newKp = commande.substring(6).toFloat();
                Serial.print("New Kp: ");
                Serial.println(newKp);
                flywheelMotor.setKp(newKp);
            }

            else if(commande.startsWith("setKi")){
                float newKi = commande.substring(6).toFloat();
                Serial.print("New Ki: ");
                Serial.println(newKi);
                flywheelMotor.setKp(newKi);
            }

            else if(commande.startsWith("setKd")){
                float newKd = commande.substring(6).toFloat();
                Serial.print("New Kd: ");
                Serial.println(newKd);
                flywheelMotor.setKp(newKd);
            }

            else
                Serial.println("Input invalid");


        }
        else
            Serial.println("Input invalid");

		commande = ""; //empty for next input
    }
}