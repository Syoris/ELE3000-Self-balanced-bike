#include "Main_Controller.h"

double Kp_v = -2500;
double Ki_v = -0;
double Kd_v = -100;

const float cutoff_freq   = 10;  //Cutoff frequency in Hz
const float sampling_time = COMPUTE_INTERVAL_ANGLE/1000000; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD2; // Order (OD1 to OD4)

MainController mainController;

MainController::MainController():_anglePID(&_currentAngle, &_accelOutput, &_targetAngle, Kp_v, Ki_v, Kd_v, DIRECT),
                                 _f(cutoff_freq, sampling_time, order){
    _targetAngle = 0;
    _Kp = Kp_v;
    _Ki = Ki_v;
    _Kd = Kd_v;
    
    _anglePID.SetMode(AUTOMATIC);
    _anglePID.SetOutputLimits(-MAX_ACCEL, MAX_ACCEL);
    _anglePID.SetTunings(_Kp, _Ki, _Kd);
    _anglePID.SetSampleTime(COMPUTE_INTERVAL_ANGLE/1000);
    _anglePID.SetComputeMode(true);
    _anglePID.toPrint = false;

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
    _anglePID.InitSpeed();
    flywheelMotor.startMotor();
    _prevComputeTime = millis();
    _prevAngle = _currentAngle;
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
    _currentAngle = _ypr[1] + ZERO_OFFSET; // to correct sensor
}

void MainController::computeCommand(){
    if(_toStabilize){

        updateAngle();
        double currentTime = millis();
        double timeChange = double(currentTime - _prevComputeTime)/1000; //Time change in seconds

        if(timeChange > COMPUTE_INTERVAL_ANGLE/1000000){
            _angVel = (_currentAngle - _prevAngle)/timeChange;
            _angVelF = _f.filterIn(_angVel);
            _anglePID.Compute(_angVelF);

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

void MainController::measureAngVel(){

}


//! Interface
double MainController::getAngle(){return _currentAngle;} 

double MainController::getTargetAngle(){return _targetAngle;}

double MainController::getAngularVel(){return _angVel;}

double MainController::getAngularVelFiltered(){return _angVelF;}

double MainController::getTargetAccel(){return _accelOutput*RAD_TO_DEG;}

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