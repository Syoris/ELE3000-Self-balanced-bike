#include "Main_Controller.h"

double Kp_v = 1000;
double Ki_v = 0;
double Kd_v = 100;

MainController mainController;

MainController::MainController():_anglePID(&_currentAngle, &_speedOutput, &_targetAngle, Kp_v, Ki_v, Kd_v, DIRECT){
    _targetAngle = 0;
    _Kp = Kp_v;
    _Ki = Ki_v;
    _Kd = Kd_v;
    
    _anglePID.SetMode(AUTOMATIC);
    _anglePID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
    _anglePID.SetTunings(_Kp, _Ki, _Kd);
    _anglePID.SetSampleTime(COMPUTE_INTERVAL_ANGLE/1000);

    flywheelMotor.setBikeAngle(&_currentAngle);

    _imuRdy = IMU_Setup();
}

void MainController::startController(){
    _toStabilize = true;
    flywheelMotor.startMotor();
}

void MainController::stopController(){
    _toStabilize = false;
    flywheelMotor.stopMotor();
    Serial.print("!");
    Serial.print(_anglePID.GetKp());
    Serial.print(", ");
    Serial.print(_anglePID.GetKi());
    Serial.print(", ");
    Serial.println(_anglePID.GetKd());
}


void MainController::updateAngle(){
    if (!_imuRdy) return; //Check IMU is working

    IMU_Compute(_ypr);
    _currentAngle = _ypr[1];
}

void MainController::computeCommand(){
    if(_toStabilize){

        updateAngle();

        if(_anglePID.Compute()){
            flywheelMotor.setTargetSpeed(_speedOutput);

            // Serial.print("Current Angle: ");
            // Serial.print(_currentAngle);
            // Serial.print("\t Flywheel speed: ");
            // Serial.println(_speedOutput);
        }
    }
}


//! Interface
double MainController::getAngle(){return _currentAngle;} 


double MainController::getTargetAngle(){return _targetAngle;}

double MainController::getKp(){return _anglePID.GetKp();}

double MainController::getKi(){return _anglePID.GetKi();}

double MainController::getKd(){return _anglePID.GetKd();}

void MainController::setTargetAngle(double newAngle){_targetAngle = newAngle;}


void MainController::setPID(){ 
    _anglePID.SetTunings(_Kp, _Ki, _Kd);
}

void MainController::setPID(double Kp, double Ki, double Kd){ 
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _anglePID.SetTunings(_Kp, _Ki, _Kd);
}

void MainController::setKp(double Kp){
    _Kp = Kp;
    setPID();
}

void MainController::setKi(double Ki){
    _Ki = Ki;
    setPID();
}

void MainController::setKd(double Kd){
    _Kd = Kd;
    setPID();
}