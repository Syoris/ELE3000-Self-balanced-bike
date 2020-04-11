#include "Motor_Controller.h"

// double Kp_m = 0.003812;
// double Ki_m = 0.228711;
// double Kd_m = -0.000111;


// Pôle à -40
double Kp_m = 0.011740;
double Ki_m = 0.500114;
double Kd_m = 0;

const float cutoff_freq_motor   = 8;  //Cutoff frequency in Hz
const float sampling_time_motor = COMPUTE_INTERVAL_ANGLE/1000000; //Sampling time in seconds.
IIR::ORDER  order_motor  = IIR::ORDER::OD1; // Order (OD1 to OD4)

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
                                _speedPID(&_speedF, &_speedCommand, &_targetSpeed, Kp_m, Ki_m, Kd_m, DIRECT),
                                _f(cutoff_freq_motor, sampling_time_motor, order_motor){
    _prevAngle = 0;
    _currentAngle = 0;
    _targetSpeed = 5000;

    _Kp = Kp_m;
    _Kd = Kd_m;
    _Ki = Ki_m;

    _speedPID.SetMode(AUTOMATIC);
    _speedPID.SetOutputLimits(-6, 6);
    _speedPID.SetTunings(_Kp, _Ki, _Kd);
    _speedPID.SetSampleTime(COMPUTE_INTERVAL/1000);
    _speedPID.SetComputeMode(false);
    _speedPID.toPrint = false;   //For debugging

    _speedMeasureTimer.priority(255);
    _speedComputeTimer.priority(254);


    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
}

void FlywheelMotor::startMotor(){
    _f.flush();
    _speedF = 0;
    _speedMeasureTimer.begin(measureSpeedTimer, SPEED_INTERVAL);
    _speedComputeTimer.begin(computeSpeedTimer, COMPUTE_INTERVAL);
    _speedPID.InitSpeed();
    _motor_enc.write(0);
    _prevAngle = 0;
    _currentAngle = 0;
    _speed = 0;
    printMotorData();
}

void FlywheelMotor::stopMotor(){
    _speedMeasureTimer.end();
    _speedComputeTimer.end();

    setMotorSpeed(0, CW);
    delay(50);

    Serial.print("!");
    Serial.print(_speedPID.GetKp(), 5);
    Serial.print(", ");
    Serial.print(_speedPID.GetKi(), 5);
    Serial.print(", ");
    Serial.println(_speedPID.GetKd(), 5);
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
    _speedF = _f.filterIn(_speed);

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
    if(dir == CCW){
        analogWrite(_pin1, 0);
        analogWrite(_pin2, speed);
    }
    else{
        analogWrite(_pin1, speed);
        analogWrite(_pin2, 0);
    }
}

//Set speed of the motor, speed > 0 => CW
void FlywheelMotor::setMotorSpeed(int speed){
    bool dir = speed < 0 ? CCW : CW;
    if(dir == CCW){
        analogWrite(_pin1, 0);
        analogWrite(_pin2, abs(speed));
    }
    else{
        analogWrite(_pin1, abs(speed));
        analogWrite(_pin2, 0);
    }
}

//Set speed of the flywheel with PID, if targetSpeed < 0 => CCW
void FlywheelMotor::computeCommand(){

    if(_speedPID.Compute(0)){
        printMotorData();
        double pwmVal = _speedCommand*V_TO_PWM;
        setMotorSpeed(pwmVal);
    }
}

void FlywheelMotor::stepReponse(double stepAmplitude){
    double stepTime = 3000; //Time in ms

    _speed = 0;
    _speedMeasureTimer.begin(measureSpeedTimer, SPEED_INTERVAL);
    _motor_enc.write(0);
    _currentAngle = 0;
    _prevAngle = 0;

    unsigned int startingTime = millis();
    unsigned int lastPrint = millis();
    unsigned int currentTime = millis();

    while(currentTime - startingTime < stepTime){
        currentTime = millis();
        setMotorSpeed(stepAmplitude*255/6, CCW);
        if(currentTime - lastPrint > COMPUTE_INTERVAL/1000){
            printMotorData();
            lastPrint = currentTime;
        }
    }
    Serial.println("*");
    stopMotor();
}

//Brake motor
void FlywheelMotor::brakeMotor(){
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, HIGH);
}

//! Interface
void FlywheelMotor::printMotorData(){
    if(!_printTextData){
        Serial.print("#");
        Serial.print(*_bikeAngle, 5);
        Serial.print(", ");
        Serial.print(_targetSpeed);
        Serial.print(", ");
        Serial.print(_speed);
        Serial.print(", ");
        Serial.print(_speedF);
        Serial.print(", ");
        Serial.print(_speedCommand);
        Serial.print(", ");
        Serial.print(_currentAngle);
        Serial.print(", ");
        Serial.print(mainController.getAngularVel());
        Serial.print(", ");
        Serial.print(mainController.getAngularVelFiltered());
        Serial.print(", ");
        Serial.print(mainController.getTargetAccel());
        Serial.print(", ");
        Serial.println(millis());
    }
    else{
        Serial.print("Time:");
        Serial.print(millis());

        Serial.print("\tBike Angle:");  //deg
        Serial.print(*_bikeAngle);

        Serial.print("\t\tGoal: ");       //deg/sec
        Serial.print(_targetSpeed);

        Serial.print("\tSpeed: ");      //deg/sec      
        Serial.print(_speed);

        Serial.print("\tPWM: ");
        Serial.print(_speedCommand);

        Serial.print("\tAngle: ");      //deg
        Serial.println(_currentAngle);
    }
    
}

//Return current motor angle
double FlywheelMotor::getAngle(){return _currentAngle;}

//Return current motor speed in deg/sec
double FlywheelMotor::getSpeed(){ return _speed;}

//Return current motor speed in rpm
double FlywheelMotor::getSpeedRPM(){ return _speed/6;}

double FlywheelMotor::getTargetSpeed(){ return _targetSpeed;}

double FlywheelMotor::getKp(){return _speedPID.GetKp();}

double FlywheelMotor::getKi(){return _speedPID.GetKi();}

double FlywheelMotor::getKd(){return _speedPID.GetKd();}


void FlywheelMotor::setTargetSpeed(double targetSpeed){ _targetSpeed = targetSpeed;}

void FlywheelMotor::setBikeAngle(double* bikeAngle){ _bikeAngle = bikeAngle;}

void FlywheelMotor::setPID(){ 
    _speedPID.SetTunings(_Kp, _Ki, _Kd);
}

void FlywheelMotor::setPID(double Kp, double Ki, double Kd){ 
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _speedPID.SetTunings(_Kp, _Ki, _Kd);
}

void FlywheelMotor::setKp(double Kp){
    _Kp = Kp;
    setPID();
}

void FlywheelMotor::setKi(double Ki){
    _Ki = Ki;
    setPID();
}

void FlywheelMotor::setKd(double Kd){
    _Kd = Kd;
    setPID();
}