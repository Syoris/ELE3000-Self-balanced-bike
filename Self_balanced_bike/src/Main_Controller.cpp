#include "Main_Controller.h"

double Kp_v = -3414.30;
double Ki_v = -3896.63;
double Kd_v = -233.80;

MainController mainController;

MainController::MainController():_anglePID(&_currentAngle, &_accelOutput, &_targetAngle, Kp_v, Ki_v, Kd_v, DIRECT){
    _targetAngle = 0;
    _Kp = Kp_v;
    _Ki = Ki_v;
    _Kd = Kd_v;
    
    _anglePID.SetMode(AUTOMATIC);
    _anglePID.SetOutputLimits(-MAX_ACCEL, MAX_ACCEL);
    _anglePID.SetTunings(_Kp, _Ki, _Kd);
    _anglePID.SetSampleTime(COMPUTE_INTERVAL_ANGLE/1000);
    _anglePID.toPrint = false;

    flywheelMotor.setBikeAngle(&_currentAngle);

    _imuRdy = IMU_Setup();

}

void MainController::startController(){
    _toStabilize = true;
    updateAngle();
    delay(100);
    updateAngle();
    flywheelMotor.setMotorSpeed(0);
    flywheelMotor.setTargetSpeed(0);
    _anglePID.InitSpeed();
    flywheelMotor.startMotor();
    _prevComputeTime = millis();
}

void MainController::stopController(){
    _toStabilize = false;
    flywheelMotor.stopMotor();
    Serial.print("!");
    Serial.print(_anglePID.GetKp(), 5);
    Serial.print(", ");
    Serial.print(_anglePID.GetKi(), 5);
    Serial.print(", ");
    Serial.println(_anglePID.GetKd(), 5);
}

void MainController::updateAngle(){
    if (!_imuRdy) return; //Check IMU is working

    IMU_Compute(_ypr);
    _currentAngle = _ypr[1] - 4*DEG_TO_RAD; // to correct sensor
}

void MainController::computeCommand(){
    if(_toStabilize){

        updateAngle();
        double currentTime = millis();

        if(_anglePID.Compute()){
            double speedInc = (_accelOutput*RAD_TO_DEG)*(double(currentTime - _prevComputeTime)/1000);
            double newSpeed = flywheelMotor.getTargetSpeed() + speedInc;
            newSpeed =  newSpeed > MAX_SPEED? MAX_SPEED  : newSpeed;
            newSpeed =  newSpeed < -MAX_SPEED? -MAX_SPEED: newSpeed;

            flywheelMotor.setTargetSpeed(newSpeed);

            // Serial.print("Current Angle: ");
            // Serial.print(_currentAngle);
            // Serial.print("\t Target accel: ");
            // Serial.print(_accelOutput);
            // Serial.print("\t Speed inc: ");
            // Serial.print(speedInc);
            // Serial.print("\t New speed: ");
            // Serial.print(newSpeed);
            // Serial.print("\t Current time: ");
            // Serial.print(currentTime);
            // Serial.print("\t Prev time: ");
            // Serial.println(_prevComputeTime);

            _prevComputeTime = currentTime;
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