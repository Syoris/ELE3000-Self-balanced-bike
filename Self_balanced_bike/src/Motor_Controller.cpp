#include "Motor_Controller.h"

double Kp = 50;
double Kd = 0;
double Ki = 0;

static void measureSpeedTimer(){
    flywheelMotor.measureSpeed();
}

static void computeSpeedTimer(){
    flywheelMotor.computeCommand();
}

FlywheelMotor flywheelMotor;

//Constructeur
FlywheelMotor::FlywheelMotor(): _motor_enc(ENC_PIN_1, ENC_PIN_2), 
                                _speedMeasureTimer(),
                                _speedComputeTimer(),
                                _speedPID(&_speed, &_speedCommand, &_targetSpeed, Kp, Ki, Kd, DIRECT){
    _prevAngle = 0;
    _currentAngle = 0;

    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;

    _speedPID.SetMode(AUTOMATIC);
    _speedPID.SetOutputLimits(-256, 256);
    _speedPID.SetTunings(_Kp, _Ki, _Kd);
    _speedPID.SetSampleTime(COMPUTE_INTERVAL/1000);

    _speedMeasureTimer.priority(255);
    _speedComputeTimer.priority(254);


    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
}

void FlywheelMotor::startMotor(){
    Serial.println("Starting timers");
    _speedMeasureTimer.begin(measureSpeedTimer, SPEED_INTERVAL);
    _speedComputeTimer.begin(computeSpeedTimer, COMPUTE_INTERVAL);
}

void FlywheelMotor::stopMotor(){
    _speedMeasureTimer.end();
    _speedComputeTimer.end();
}


//Return current motor angle
double FlywheelMotor::getAngle(){
    return _currentAngle;
}

//Return current motor speed in deg/sec
double FlywheelMotor::getSpeed(){
    return _speed;
}

//Return current motor speed in rpm
double FlywheelMotor::getSpeedRPM(){
    return _speed/6;
}

//Return encoder value
double FlywheelMotor::readAngle(){
    return _motor_enc.read()*COUNT_TO_ANGLE;
}

//Calculate motor speed
void FlywheelMotor::measureSpeed(){    
    _prevAngle = _currentAngle;
    _currentAngle = readAngle();


    _speed = ((_currentAngle - _prevAngle) * USEC_TO_SEC) / SPEED_INTERVAL;

    if(DEBUG_MOTOR){
        Serial.print("Delta angle: ");
        Serial.print(_currentAngle - _prevAngle);
        Serial.print("\t Speed (deg/sec): ");
        Serial.print(_speed);
        Serial.print("\t Speed (rpm): ");
        Serial.println(_speed/6);
    }
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

void FlywheelMotor::setTargetSpeed(double targetSpeed){
    _targetSpeed = targetSpeed;
}

//Set speed of the flywheel with PID, if targetSpeed < 0 => CCW
void FlywheelMotor::computeCommand(){

    if(_speedPID.Compute()){
        Serial.print("#");
        Serial.print(_targetSpeed);
        Serial.print(", ");
        Serial.print(_speed);
        Serial.print(", ");
        Serial.println(_speedCommand);

        // Serial.print("Target speed: ");
        // Serial.print(_targetSpeed);
        // Serial.print("\tCurrent speed: ");
        // Serial.print(_speed);
        // Serial.print("\tCommand: ");
        // Serial.println(_speedCommand);

        bool dir = _speedCommand < 0 ? CCW : CW;
        if(dir == CW){
            analogWrite(_pin1, 0);
            analogWrite(_pin2, _speedCommand);
        }
        else{
            analogWrite(_pin1, _speedCommand);
            analogWrite(_pin2, 0);
        }
    }
}

void FlywheelMotor::setPID(double Kp, double Ki, double Kd){
    _speedPID.SetTunings(Kp, Kd, Ki);
}

//Brake motor
void FlywheelMotor::brakeMotor(){
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, HIGH);
}