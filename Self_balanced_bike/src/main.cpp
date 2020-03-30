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
unsigned int prevTimeCommand = 0;
unsigned int prevTimeAccel = 0;
unsigned long currentTime = 0;


bool motorOn = false;
bool followAccel = false;
bool toStabilise = false;

unsigned int computInt;
double speedInc;

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
    if(currentTime - prevTimeCommand > IR_INTERVAL){
        prevTimeCommand = currentTime;
        //readRemote();
	    readSerial();
    }

    //! To go to a specific acceleration
    if(followAccel){
        if(currentTime - prevTimeAccel < 1000)
            goToAccel();
        else{
            Serial.println("Timeout");
            followAccel = false;
            stopController();
            Serial.println("*");
        }
    }

    //! To stabilize the bike
    if(toStabilise)
        mainController.computeCommand();

    // mainController.updateAngle();
    // Serial.println(mainController.getAngle());
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
    motorOn = false;
    followAccel = false;
    toStabilise = false;
    goalAccel = 0;
    mainController.stopController();
}

// Fonction de test
void goToAccel(){

    if(currentTime - prevTime > computInt){
        double newSpeed = flywheelMotor.getTargetSpeed() + speedInc;
        flywheelMotor.setTargetSpeed(newSpeed);
        prevTime = currentTime;
        // Serial.print("Goal accel: ");
        // Serial.print(goalAccel);
        // Serial.print("\t Compute int: ");
        // Serial.print(computInt);
        // Serial.print("\t Speed inc:");
        // Serial.print(speedInc);
        // Serial.print("\t New speed:");
        // Serial.println(newSpeed);
    }
}

void start(){
    motorOn = true;
    flywheelMotor.startMotor();
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

            // Commandes du vélo
            if(commande == "stabilize"){
                toStabilise = true;
                startController();
            }

            else if(commande.startsWith("setKpV")){
                double newKp = commande.substring(7).toFloat();
                mainController.setKp(newKp);
                Serial.print("New Kp for bike: ");
                Serial.println(mainController.getKp());
            }

            else if(commande.startsWith("setKiV")){
                double newKi = atof(commande.substring(7).c_str());
                mainController.setKi(newKi);
                Serial.print("New Ki for bike: ");
                Serial.println(mainController.getKi());
            }

            else if(commande.startsWith("setKdV")){
                double newKd = atof(commande.substring(7).c_str());
                mainController.setKd(newKd);
                Serial.print("New Kd for bike: ");
                Serial.println(mainController.getKd());
            }


            // Commandes du moteur
            else if(commande == "On"){ start();}

            else if(commande == "Off"){stopController();}

            else if(commande.startsWith("setSpeed")){
                float newSpeed = commande.substring(8).toFloat();              
                Serial.print("New speed: ");
                Serial.println(newSpeed);
                flywheelMotor.setTargetSpeed(newSpeed);
            }

            else if(commande.startsWith("step")){
                float stepVoltage = commande.substring(5).toFloat();
                Serial.print("Step of ");
                Serial.println(stepVoltage);
                flywheelMotor.stepReponse(stepVoltage);
            }

            else if(commande.startsWith("ramp")){
                float accel = commande.substring(5).toFloat();
                start();
                flywheelMotor.setTargetSpeed(0);
                Serial.print("Going to [deg/sec^2]: ");
                Serial.println(accel);
                goalAccel = accel;
                computInt = COMPUTE_INTERVAL/1000;
                speedInc = goalAccel/1000*computInt;
                followAccel = true;
                prevTimeAccel = millis();
                currentTime = millis();
            }

            else if(commande.startsWith("setKpM")){
                double newKp = atof(commande.substring(7).c_str());
                flywheelMotor.setKp(newKp);
                Serial.print("New Kp: ");
                Serial.println(flywheelMotor.getKp());
            }

            else if(commande.startsWith("setKiM")){
                double newKi = atof(commande.substring(7).c_str());
                flywheelMotor.setKi(newKi);
                Serial.print("New Ki: ");
                Serial.println(flywheelMotor.getKi());
            }

            else if(commande.startsWith("setKdM")){
                double newKd = atof(commande.substring(7).c_str());
                flywheelMotor.setKd(newKd);
                Serial.print("New Kd: ");
                Serial.println(flywheelMotor.getKd());
            }
            
            else
                Serial.println("Input invalid");


        }
        else
            Serial.println("Input invalid");

		commande = ""; //empty for next input
    }
}