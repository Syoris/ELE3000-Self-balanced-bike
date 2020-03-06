#include "Motor_Controller.h"

static void measureSpeedTimer(){
    flywheelMotor.measureSpeed();
}

FlywheelMotor flywheelMotor;

//Constructeur
FlywheelMotor::FlywheelMotor(): _motor_enc(ENC_PIN_1, ENC_PIN_2), 
                                _speedTimer(), 
                                _speedPID(&_speed, &_speedCommand, &_targetSpeed, _Kp, _Kd, _Ki, DIRECT){
    _prevAngle = 0;
    _currentAngle = 0;

    _speedTimer.begin(measureSpeedTimer, 50);

    _Kp = 4;
    _Kd = 0;
    _Ki = 1;

    _speedPID.SetOutputLimits(0, 1023);
    _speedPID.SetTunings(_Kp, _Kd, _Ki);


    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
}

//Return current motor angle
double FlywheelMotor::getAngle(){
    return _currentAngle;
}

//Return current motor speed
double FlywheelMotor::getSpeed(){
    return _speed;
}

//Return encoder value
double FlywheelMotor::readAngle(){
    return _motor_enc.read()*COUNT_TO_ANGLE;
}

//Calculate motor speed
void FlywheelMotor::measureSpeed(){
    _prevAngle = _currentAngle;
    _currentAngle = readAngle();

    _speed = (_currentAngle - _prevAngle)/SPEED_INTERVAL*USEC_TO_SEC;
}

//Set speed of the motor, dir:CW or CCW
void FlywheelMotor::setMotorSpeed(int speed, bool dir){
    if(dir == CW){
        analogWrite(_pin1, 0);
        analogWrite(_pin2, speed);
    }
    else{
        analogWrite(_pin1, speed);
        analogWrite(_pin2, 0);
    }
}

//Set speed of the flywheel with PID, if targetSpeed < 0 => CCW
void FlywheelMotor::setMotorSpeedPID(double targetSpeed){
    _targetSpeed = targetSpeed;
    bool dir = _targetSpeed < 0 ? CCW : CW;

    _speedPID.Compute();

    if(dir == CW){
        analogWrite(_pin1, 0);
        analogWrite(_pin2, _speedCommand);
    }
    else{
        analogWrite(_pin1, _speedCommand);
        analogWrite(_pin2, 0);
    }
}

//Brake motor
void FlywheelMotor::brakeMotor(){
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, HIGH);
}