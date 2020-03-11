#include <Arduino.h>

#include <IMU.h>
#include <Motor_Controller.h>
#include <IR_Receiver.h>

#include "Servo.h"

// Define
#define LED_PIN 13

void readSerial();
void start();
void stop();
void IMU_test();
void goToAccel();

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
    flywheelMotor.setTargetSpeed(1000);

}


// Variables
double angle = 0;
double goalAccel = 0;
String commande;

int max_speed = 256;
unsigned int prevTime = 0;

bool motorOn = false;
bool followAccel = false;
bool toStabilise = false;


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {

    //! Read Serial Input
	readSerial();

    


    //! To go to a specific acceleration
    if(followAccel)
        goToAccel();

}

void goToAccel(){
    static double speedInc = goalAccel/(COMPUTE_INTERVAL/USEC_TO_SEC);
    unsigned int currentTime = millis();
    if(currentTime-prevTime > COMPUTE_INTERVAL){
        flywheelMotor.setTargetSpeed(flywheelMotor.getTargetSpeed() + speedInc);
        prevTime = currentTime;
    }
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

void start(){
    motorOn = true;
    flywheelMotor.startMotor();
}

void stop(){
    motorOn = false;
    followAccel = false;
    toStabilise = false;
    flywheelMotor.stopMotor();
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

            if(commande == "On"){ start();}

            else if(commande == "Off"){stop();}

            else if(commande == "stabilise"){
                toStabilise = true;
                start();
            }

            else if(commande == "accel"){
                float accel = commande.substring(6).toFloat();
                start();
                Serial.print("Going to [deg/sec^2]: ");
                Serial.println(accel);
                goalAccel = accel;
                followAccel = true;
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
                flywheelMotor.setKi(newKi);
            }

            else if(commande.startsWith("setKd")){
                float newKd = commande.substring(6).toFloat();
                Serial.print("New Kd: ");
                Serial.println(newKd);
                flywheelMotor.setKd(newKd);
            }

            else if(commande.startsWith("step")){
                float stepVoltage = commande.substring(5).toFloat();
                Serial.print("Step of ");
                Serial.println(stepVoltage);
                flywheelMotor.stepReponse(stepVoltage);
            }
            
            else
                Serial.println("Input invalid");


        }
        else
            Serial.println("Input invalid");

		commande = ""; //empty for next input
    }
}