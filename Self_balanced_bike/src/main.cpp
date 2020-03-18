#include <Arduino.h>

#include <IMU.h>
#include <Motor_Controller.h>
#include <Main_Controller.h>
#include <IR_Receiver.h>

#include "Servo.h"

// Define
#define LED_PIN 13
#define IR_INTERVAL 250 //In ms

// Associations bouttons
#define STABILIZE OFF

// Prototypes
void readSerial();
void readRemote();

void start();
void stop();
void startController();
void stopController();

void IMU_test();
void goToAccel();

// Variables
double goalAccel;   //To test motor acceleration
String commande;    //Command read through serial port

unsigned int prevTime = 0;
unsigned long currentTime = 0;


bool motorOn = false;
bool followAccel = false;
bool toStabilise = false;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    //! Initialize serial communication
    Serial.begin(9600);

    //! Configure LED for output
    IR_Setup();

    // pinMode(LED_PIN, OUTPUT);
    // digitalWrite(LED_PIN, HIGH);

}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    currentTime = millis();

    //! Read Serial Input
	//readSerial();
    if(currentTime - prevTime > IR_INTERVAL){
        prevTime = currentTime;
        readRemote();
    }

    //! To go to a specific acceleration
    if(followAccel)
        goToAccel();

    //! To stabilize the bike
    if(toStabilise)
        mainController.computeCommand();
    
}

void readRemote(){
    unsigned long val = IR_receive();
    switch (val)
    {
    case STABILIZE:
        Serial.println("Toggle stabilization");
        toStabilise = !toStabilise;
        if(toStabilise)
            startController();
        else
            stopController();
        break;

    case UP:
        Serial.println("UP");
        break;

    default:
        break;
    }
}

void startController(){
    mainController.startController();
}

void stopController(){
    mainController.stopController();
}

// Fonction de test
void goToAccel(){
    static unsigned int computInt = COMPUTE_INTERVAL/1000;
    static double speedInc = goalAccel/1000*computInt;

    unsigned int currentTime = millis();
    if(currentTime-prevTime > computInt){
        flywheelMotor.setTargetSpeed(flywheelMotor.getTargetSpeed() + speedInc);
        prevTime = currentTime;
    }
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

            else if(commande.startsWith("accel")){
                float accel = commande.substring(6).toFloat();
                start();
                flywheelMotor.setTargetSpeed(0);
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
                double newKp = atof(commande.substring(6).c_str());
                flywheelMotor.setKp(newKp);
                Serial.print("New Kp: ");
                Serial.println(flywheelMotor.getKp());
            }

            else if(commande.startsWith("setKi")){
                double newKi = atof(commande.substring(6).c_str());
                flywheelMotor.setKi(newKi);
                Serial.print("New Ki: ");
                Serial.println(flywheelMotor.getKi());
            }

            else if(commande.startsWith("setKd")){
                double newKd = atof(commande.substring(6).c_str());
                flywheelMotor.setKd(newKd);
                Serial.print("New Kd: ");
                Serial.println(flywheelMotor.getKd());
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