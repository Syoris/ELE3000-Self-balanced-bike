#include "Main_Controller.h"

double Kp_v = -4300;
double Ki_v = -0;
double Kd_v = -320;

const float cutoff_freq_speed   = 21;  //Cutoff frequency in Hz
IIR::ORDER  order_speed  = IIR::ORDER::OD1; // Order (OD1 to OD4)

const float cutoff_freq_angle   = 10;  //Cutoff frequency in Hz
IIR::ORDER  order_angle  = IIR::ORDER::OD2; // Order (OD1 to OD4)

MainController mainController;

MainController::MainController():_speedFilter(cutoff_freq_speed, SPEED_MEASURE_INTERVAL/1000, order_speed),
                                 _angleFilter(cutoff_freq_angle, ANGLE_MEASURE_INTERVAL/1000, order_angle){
    _targetAngle = 0;
    _Kp = Kp_v;
    _Ki = Ki_v;
    _Kd = Kd_v;

    _zeroOffset = ZERO_OFFSET;
    _deadZone = DEAD_ZONE;
    _inDeadZone = false;
    _inDanger = false;

}

void MainController::init(){
    _imuRdy = IMU_Setup();
}

void MainController::startController(){
    _toStabilize = true;
    _speedFilter.flush();
    _angleFilter.flush();

    _angVel = 0;
    _angVelRaw = 0;
    _prevAngVel = 0;
    _outputSum = 0;
    _targetAngle = 0;
    _inDanger = false;

    unsigned int curTime = millis();
    while(millis() - curTime < 250)
        updateAngle();

    flywheelMotor.setMotorSpeed(0);
    flywheelMotor.setTargetSpeed(0);
    flywheelMotor.startMotor();

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
    _currentAngle = _ypr[1] + _zeroOffset; // to correct sensor
}

void MainController::measureSpeed(double timeInt){
    _angVelRaw = (_currentAngle - _prevAngle)/timeInt;  //Compute angular velocity
    _angVel = _speedFilter.filterIn(_angVelRaw);        //Filter velocity
    _prevAngle = _currentAngle;
}

void MainController::computeCommand(double timeInt){
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

    double error = _targetAngle - _currentAngle;
    double output;

    //Compute PID
    _outputSum += _Ki * error * timeInt;   
    output = _Kp * error + _outputSum - _Kd * _angVel; //output = Kp*Error - Kd*Angular speed

    //Check for the dead zone: if angle has been in +/- DeadZone range for more than 100ms, output = 0
    if(abs(_currentAngle) < DEAD_ZONE && !_inDeadZone){
        _inDeadZone = true;
        _deadTime = millis();
    }

    if(abs(_currentAngle) > DEAD_ZONE)
        _inDeadZone = false;
    
    if(_inDeadZone && millis() - _deadTime > 100)
        output = 0;

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

float MainController::getZeroOffset(){return _zeroOffset*RAD_TO_DEG;}

double MainController::getDeadZone(){return _deadZone*RAD_TO_DEG;}

double MainController::getKp(){return _Kp;}

double MainController::getKi(){return _Ki;}

double MainController::getKd(){return _Kd;}

void MainController::setTargetAngle(double newAngle){_targetAngle = newAngle;}

void MainController::setKp(double Kp){_Kp = Kp;}

void MainController::setKi(double Ki){_Ki = Ki;}

void MainController::setKd(double Kd){_Kd = Kd;}

//Add offset to current offset
void MainController::setZeroOffset(float newOffset){ _zeroOffset += newOffset*DEG_TO_RAD;}

//Add value to current offset
void MainController::setDeadZone(double newDeadZone){ _deadZone += newDeadZone*DEG_TO_RAD;}