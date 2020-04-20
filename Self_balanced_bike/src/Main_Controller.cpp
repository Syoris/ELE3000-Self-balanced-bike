#include "Main_Controller.h"

double Kp_v = -4300;
double Ki_v = -0;
double Kd_v = -320;

const float cutoff_freq_speed   = 21;  //Cutoff frequency in Hz
IIR::ORDER  order_speed  = IIR::ORDER::OD1; // Order (OD1 to OD4)

MainController mainController;

MainController::MainController():_speedFilter(cutoff_freq_speed, SPEED_MEASURE_INTERVAL/1000, order_speed){
    _targetAngle = 0;
    _Kp = Kp_v;
    _Ki = Ki_v;
    _Kd = Kd_v;

    _inDanger = false;

}

void MainController::init(){
    _imuRdy = IMU_Setup();
}

void MainController::startController(){
    _toStabilize = true;
    _speedFilter.flush();
    
    // Reset variables
    _angVel = 0;
    _angVelRaw = 0;
    _prevAngVel = 0;
    _outputSum = 0;
    _targetAngle = 0;
    _inDanger = false;

    // Wait 250ms to make sure the angle is stable
    unsigned int curTime = millis();
    while(millis() - curTime < 250)
        updateAngle();

    // Initialize flywheel
    flywheelMotor.setMotorSpeed(0);
    flywheelMotor.setTargetSpeed(0);
    flywheelMotor.startMotor();

    // Set time variables
    _prevComputeTime = millis();
    _prevAngleTime = _prevComputeTime;
    _prevSpeedTime = _prevComputeTime;
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
    _currentAngle = _ypr[1]; // to correct sensor
}

void MainController::measureSpeed(double timeInt){
    _angVelRaw = (_currentAngle - _prevAngle)/timeInt;  //Compute angular velocity
    _angVel = _speedFilter.filterIn(_angVelRaw);        //Filter velocity
    _prevAngle = _currentAngle;
}

void MainController::computeCommand(double timeInt){
    
    // Flywheel speed limit algorithm
    double mSpeed = flywheelMotor.getSpeed();
    if(mSpeed > SPD_UP_THRES && !_inDanger){
        _inDanger = true;
        _targetAngle = -DANGER_ANGLE_TARGET;
    }
    else if(mSpeed < -SPD_UP_THRES && !_inDanger){
        _inDanger = true;
        _targetAngle = DANGER_ANGLE_TARGET;
    }
    else if(!_inDanger){
        _inDanger = false;
        _targetAngle = 0.0;
    }

    if(_inDanger && abs(mSpeed) < SPD_DWN_TRHES){
        _inDanger = false;
        _targetAngle = 0;
    }

    //Compute PID
    double error = _targetAngle - _currentAngle;
    double output;

    _outputSum += _Ki * error * timeInt;   
    output = _Kp * error + _outputSum - _Kd * _angVel; //output = Kp*Error - Kd*Angular speed

    //Set the output
    _accelOutput = output;

    double speedInc = (_accelOutput*RAD_TO_DEG)*timeInt;
    double newSpeed = flywheelMotor.getTargetSpeed() + speedInc;

    newSpeed =  newSpeed > MAX_SPEED? MAX_SPEED  : newSpeed;
    newSpeed =  newSpeed < -MAX_SPEED? -MAX_SPEED: newSpeed;

    flywheelMotor.setTargetSpeed(newSpeed);
}

void MainController::stabilise(){
    if(_toStabilize){
        unsigned long currentTime = millis();
        unsigned long timeChangeAngle = (currentTime - _prevAngleTime); //Time change for angle in mS
        double timeChangeSpeed = (currentTime - _prevSpeedTime); //Time change for speed in mS
        double timeChangeCompute = (currentTime - _prevComputeTime); //Time change for PID in mS
        
        if(timeChangeAngle >= ANGLE_MEASURE_INTERVAL){
            updateAngle();
            _prevAngleTime = currentTime;
        }
        
        if(timeChangeSpeed >= SPEED_MEASURE_INTERVAL){
            measureSpeed(timeChangeSpeed/1000.0);
            _prevSpeedTime = currentTime;
        }

        if(timeChangeCompute >= COMPUTE_INTERVAL_ANGLE){
            computeCommand(timeChangeCompute/1000);
            _prevComputeTime = currentTime;
        }

        if(currentTime - _prevPrintTime >= SEND_DATA_INTERVAL){
            flywheelMotor.printMotorData();
            _prevPrintTime = currentTime;
        }
    }
}


//! Interface
double MainController::getAngle(){return _currentAngle;} 

double MainController::getAngleRaw(){return _currentAngleRaw;} 

double MainController::getAngularVel(){return _angVel;}

double MainController::getAngularVelRaw(){return _angVelRaw;}

double MainController::getTargetAngle(){return _targetAngle;}

double MainController::getTargetAccel(){return _accelOutput*RAD_TO_DEG;}

double MainController::getKp(){return _Kp;}

double MainController::getKi(){return _Ki;}

double MainController::getKd(){return _Kd;}

void MainController::setTargetAngle(double newAngle){_targetAngle = newAngle;}

void MainController::setKp(double Kp){_Kp = Kp;}

void MainController::setKi(double Ki){_Ki = Ki;}

void MainController::setKd(double Kd){_Kd = Kd;}