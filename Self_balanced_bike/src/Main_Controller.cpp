#include "Main_Controller.h"

double Kp_v = -2500;
double Ki_v = -0;
double Kd_v = -100;

const float cutoff_freq   = 10;  //Cutoff frequency in Hz
const float sampling_time = COMPUTE_INTERVAL_ANGLE/1000000; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD2; // Order (OD1 to OD4)

MainController mainController;

MainController::MainController():_f(cutoff_freq, sampling_time, order){
    _targetAngle = 0;
    _Kp = Kp_v;
    _Ki = Ki_v;
    _Kd = Kd_v;

    flywheelMotor.setBikeAngle(&_currentAngle);

    _imuRdy = IMU_Setup();

}

void MainController::startController(){
    _toStabilize = true;
    _f.flush();
    _angVelF = 0;

    unsigned int curTime = millis();
    while(millis() - curTime < 250)
        updateAngle();

    flywheelMotor.setMotorSpeed(0);
    flywheelMotor.setTargetSpeed(0);
    flywheelMotor.startMotor();

    _prevComputeTime = millis();
    _prevAngle = _currentAngle;
}

void MainController::stopController(){
    _toStabilize = false;
    flywheelMotor.stopMotor();
    Serial.print("!");
    Serial.print(_Kp, 5);
    Serial.print(", ");
    Serial.print(_Ki, 5);
    Serial.print(", ");
    Serial.println(_Kd, 5);
}

void MainController::updateAngle(){
    if (!_imuRdy) return; //Check IMU is working

    IMU_Compute(_ypr);
    _currentAngle = _ypr[1] + ZERO_OFFSET; // to correct sensor
}

void MainController::computeCommand(){
    if(_toStabilize){

        updateAngle();

        unsigned long currentTime = millis();
        unsigned long timeChange = (currentTime - _prevComputeTime)/1000; //Time change in seconds

        if(timeChange > COMPUTE_INTERVAL_ANGLE/1000000){
            _angVel = (_currentAngle - _prevAngle)/timeChange;  //Compute angular velocity
            _angVelF = _f.filterIn(_angVel);                    //Filter velocity

            double error = _targetAngle - _currentAngle;
            double output;


            //Compute PD   
            output = _Kp * error - _Kd * _angVelF; //output = Kp*Error - Kd*Angular speed
            _accelOutput = output;

            double speedInc = (_accelOutput*RAD_TO_DEG)*timeChange;
            double newSpeed = flywheelMotor.getTargetSpeed() + speedInc;

            newSpeed =  newSpeed > MAX_SPEED? MAX_SPEED  : newSpeed;
            newSpeed =  newSpeed < -MAX_SPEED? -MAX_SPEED: newSpeed;

            flywheelMotor.setTargetSpeed(newSpeed);

            _prevComputeTime = currentTime;
            _prevAngle = _currentAngle;
        }
    }
}


//! Interface
double MainController::getAngle(){return _currentAngle;} 

double MainController::getTargetAngle(){return _targetAngle;}

double MainController::getAngularVel(){return _angVel;}

double MainController::getAngularVelFiltered(){return _angVelF;}

double MainController::getTargetAccel(){return _accelOutput*RAD_TO_DEG;}

double MainController::getKp(){return _Kp;}

double MainController::getKi(){return _Ki;}

double MainController::getKd(){return _Kd;}

void MainController::setTargetAngle(double newAngle){_targetAngle = newAngle;}

void MainController::setKp(double Kp){_Kp = Kp;}

void MainController::setKi(double Ki){_Ki = Ki;}

void MainController::setKd(double Kd){_Kd = Kd;}