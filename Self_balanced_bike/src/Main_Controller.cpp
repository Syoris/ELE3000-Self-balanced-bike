#include "Main_Controller.h"

double Ki_v = 0.7075;
double Kp_v = 0.0672;
double Kd_v = 0.0034;

MainController mainController;

//TODO
void computeAngleTimer(){

}

//TODO
MainController::MainController():_anglePID(&_currentAngle, &_speedOutput, &_targetAngle, Kp_v, Ki_v, Kd_v, DIRECT),
                                _computeTimer(){
    _targetAngle = 0;
    _Kp = Kp_v;
    _Ki = Ki_v;
    _Kd = Kd_v;
    
    _anglePID.SetMode(AUTOMATIC);
    _anglePID.SetOutputLimits(0, 256);
    _anglePID.SetTunings(_Kp, _Ki, _Kd);
    _anglePID.SetSampleTime(COMPUTE_INTERVAL_ANGLE/1000);

    _computeTimer.priority(255);


    //TODO Ajouter init du IMU
}


void MainController::startController(){
    _computeTimer.begin(computeAngleTimer, COMPUTE_INTERVAL_ANGLE);

}

void MainController::stopController(){
    _computeTimer.end();
}


//TODO
void MainController::readAngle(){

}

//TODO
void MainController::computeCommand(){

}


//! Interface
double MainController::getAngle(){return _currentAngle;} 


double MainController::getTargetAngle(){return _targetAngle;}


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