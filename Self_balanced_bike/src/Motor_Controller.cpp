#include "Motor_Controller.h"

static void measureSpeedTimer(){
    flywheelMotor.measureSpeed();
}

FlywheelMotor flywheelMotor;

FlywheelMotor::FlywheelMotor():_motor_enc(ENC_PIN_1, ENC_PIN_2), _speedTimer(){
    _prevAngle = 0;
    _currentAngle = 0;

    _speedTimer.begin(measureSpeedTimer, 50);

    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
}

double FlywheelMotor::getAngle(){
    return _currentAngle;
}

double FlywheelMotor::getSpeed(){
    return _speed;
}

double FlywheelMotor::readAngle(){
    return _motor_enc.read()*COUNT_TO_ANGLE;
}

void FlywheelMotor::measureSpeed(){
    _prevAngle = _currentAngle;
    _currentAngle = readAngle();

    _speed = (_currentAngle - _prevAngle)/SPEED_INTERVAL*USEC_TO_SEC;
}

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

void FlywheelMotor::brakeMotor(){
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, HIGH);
}