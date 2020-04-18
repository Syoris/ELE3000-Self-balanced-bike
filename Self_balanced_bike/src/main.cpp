#include <Arduino.h>

#include <IMU.h>
#include <Motor_Controller.h>
#include <Main_Controller.h>
#include <IR_Receiver.h>

#include "Servo.h"

// Define
#define LED_PIN 13
#define SERVO_PIN 10
#define IR_INTERVAL 100 //In ms
#define SERVO_MAX_SPEED 3

// Associations bouttons
#define STABILIZE OFF
#define SPEED_UP UP
#define SPEED_DWN DOWN

// Prototypes
void readSerial();
void readRemote();

void start();
void stop();
void startController();
void stopController();

void IMU_test();
void goToAccel();

void setServoSpeed(int speed);
float str2float(String val);

// Variables
double goalAccel;   //To test motor acceleration
String commande;    //Command read through serial port

unsigned int prevTime = 0;
unsigned int prevTimeCommand = 0;
unsigned int prevTimeAccel = 0;
unsigned int prevTimeAngle = 0;
unsigned long currentTime = 0;


bool motorOn = false;
bool followAccel = false;
bool toStabilise = false;
bool toCheckAngle = false;

double computInt;
double speedInc;

Servo servo;
int servoSpeed = 0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    //! Initialize serial communication
    Serial.begin(9600);
    Serial.println("setup");

    //! Init 
    mainController.init();    
    IR_Setup();

    servo.attach(SERVO_PIN);
    setServoSpeed(servoSpeed);

    digitalWrite(LED_PIN, LOW);

}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    currentTime = millis();

    //! Read Serial Input
    if(currentTime - prevTimeCommand > IR_INTERVAL){
        prevTimeCommand = currentTime;
        readRemote();
	    readSerial();
    }

    //! To go to a specific acceleration
    if(followAccel){
        if(currentTime - prevTimeAccel < 1000)
            goToAccel();
        else{
            Serial.println("Timeout");
            followAccel = false;
            Serial.println("*");
            stopController();

            Serial.print("Comput int: ");
            Serial.print(computInt);
            Serial.print("\tSpeed inc: ");
            Serial.println(speedInc);
        }
    }

    //! To stabilize the bike
    if(toStabilise){
        mainController.stabilise();
    }
    
    if(toCheckAngle){
        mainController.updateAngle();
        if(currentTime - prevTimeAngle > 50){
            double angle = mainController.getAngle();
            if(abs(angle) < 0.05*DEG_TO_RAD)
                digitalWrite(13, HIGH);
            else
                digitalWrite(13, LOW);

            Serial.print("#");
            Serial.println(angle, 5);
            prevTimeAngle = currentTime;
        }
    }    

}


void readRemote(){
    unsigned long val = IR_receive();
    switch (val)
    {
    case STABILIZE:
        delay(100);
        Serial.println("Toggle stabilization");
        toStabilise = !toStabilise;
        servoSpeed = 0;
        setServoSpeed(servoSpeed);

        if(toStabilise)
            startController();
        else
            stopController();
        break;

    case SPEED_UP:
        servoSpeed += 1;
        servoSpeed = (servoSpeed > SERVO_MAX_SPEED) ? SERVO_MAX_SPEED : servoSpeed;
        Serial.print("Speed up: ");
        Serial.println(servoSpeed);
        setServoSpeed(servoSpeed);
        break;
    
    case SPEED_DWN:
        servoSpeed -= 1;
        servoSpeed = (servoSpeed < -SERVO_MAX_SPEED) ? -SERVO_MAX_SPEED : servoSpeed;
        Serial.print("Speed down: ");
        Serial.println(servoSpeed);
        setServoSpeed(servoSpeed);
        break;

    default:
        break;
    }
}

void setServoSpeed(int speed){
    static int speedVal = 0;
    static int servoSpeeds[6] = {30, 15, 7, 7, 15, 30};
    switch(speed)
    {
    case -3:
        speedVal = 90 + servoSpeeds[5];
        break;
    
    case -2:
        speedVal = 90 + servoSpeeds[4];
        break;
    
    case -1:
        speedVal = 90 + servoSpeeds[3];
        break;

    case 0:
        speedVal = 90;
        break;

    case 1:
        speedVal = 90 - servoSpeeds[2];
        break;
    
    case 2:
        speedVal = 90 - servoSpeeds[1];
        break;

    case 3:
        speedVal = 90 - servoSpeeds[0];
        break;

    default:
        speedVal = 90;
        break;
    }
    
    servo.write(speedVal);
}

void startController(){
    mainController.startController();
}

void stopController(){
    digitalWrite(LED_PIN, LOW);
    motorOn = false;
    followAccel = false;
    toStabilise = false;
    toCheckAngle = false;
    goalAccel = 0;
    mainController.stopController();

}

// Fonction de test
void goToAccel(){
    currentTime = millis();
    mainController.updateAngle();
    if(currentTime - prevTime > computInt){
        speedInc = goalAccel*(double(currentTime-prevTime)/1000);
        double newSpeed = flywheelMotor.getTargetSpeed() + speedInc;
        flywheelMotor.setTargetSpeed(newSpeed);
        flywheelMotor.printMotorData();
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
    delay(50);
    mainController.updateAngle();
    delay(100);
    mainController.updateAngle();
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
            if(commande == "stabilize" || commande == "s"){
                toStabilise = true;
                startController();
            }

            // Commandes du moteur
            else if(commande == "On"){ start();}

            else if(commande == "Off" or commande == "O"){stopController();}

            else if(commande.startsWith("setSpeed")){
                float newSpeed = commande.substring(8).toFloat();              
                flywheelMotor.setTargetSpeed(newSpeed);
                Serial.print("New speed: ");
                Serial.println(flywheelMotor.getTargetSpeed());
            }

            else if(commande.startsWith("step")){
                float stepVoltage = commande.substring(5).toFloat();
                Serial.print("Step of ");
                Serial.println(stepVoltage);
                flywheelMotor.stepReponse(stepVoltage);
            }

            else if(commande.startsWith("ramp")){
                float accel = commande.substring(5).toFloat();
                flywheelMotor.setTargetSpeed(0);
                start();
                Serial.print("Going to [deg/sec^2]: ");
                Serial.println(accel);
                goalAccel = accel;
                // computInt = COMPUTE_INTERVAL/1000;
                // speedInc = (goalAccel*computInt)/1000;
                followAccel = true;
                currentTime = millis();
                prevTimeAccel = currentTime;
                prevTime = currentTime;

            }

            else if(commande == "checkAngle"){toCheckAngle = true;}

            else if(commande.startsWith("setKpV")){
                double newKp = commande.substring(7).toFloat();
                mainController.setKp(newKp);
                Serial.print("New Kp for bike: ");
                Serial.println(mainController.getKp(), 5);
            }

            else if(commande.startsWith("setKiV")){
                double newKi = atof(commande.substring(7).c_str());
                mainController.setKi(newKi);
                Serial.print("New Ki for bike: ");
                Serial.println(mainController.getKi(), 5);
            }

            else if(commande.startsWith("setKdV")){
                double newKd = atof(commande.substring(7).c_str());
                mainController.setKd(newKd);
                Serial.print("New Kd for bike: ");
                Serial.println(mainController.getKd(), 5);
            }

            else if(commande.startsWith("setKpM")){
                double newKp = atof(commande.substring(7).c_str());
                flywheelMotor.setKp(newKp);
                Serial.print("New Kp: ");
                Serial.println(flywheelMotor.getKp(), 5);
        
            }

            else if(commande.startsWith("setKiM")){
                double newKi = atof(commande.substring(7).c_str());
                flywheelMotor.setKi(newKi);
                Serial.print("New Ki: ");
                Serial.println(flywheelMotor.getKi(), 5);
            }

            else if(commande.startsWith("setKdM")){
                double newKd = atof(commande.substring(7).c_str());
                flywheelMotor.setKd(newKd);
                Serial.print("New Kd: ");
                Serial.println(flywheelMotor.getKd(), 5);
            }
            
            else if(commande.startsWith("setOffset")){
                float newOffset = atof(commande.substring(10).c_str());
                mainController.setZeroOffset(newOffset);
                Serial.print("New Offset: ");
                Serial.println(mainController.getZeroOffset(), 4);
            }
            
            else if(commande.startsWith("setDeadZone")){
                double newDeadZone = atof(commande.substring(12).c_str());
                mainController.setDeadZone(newDeadZone);
                Serial.print("New DeadZone: ");
                Serial.println(mainController.getDeadZone());
            }

            else
                Serial.println("Input invalid");


        }
        else
            Serial.println("Input invalid");

		commande = ""; //empty for next input
    }
}