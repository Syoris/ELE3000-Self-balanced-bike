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
                                _f(cutoff_freq_motor, sampling_time_motor, order_motor){
    _prevAngle = 0;
    _currentAngle = 0;
    _targetSpeed = 5000;

    _Kp = Kp_m;
    _Kd = Kd_m;
    _Ki = Ki_m;
    _KiSamp = _Ki*COMPUTE_INTERVAL/1000000;     //Ki * computeTime in sec

    _speedMeasureTimer.priority(255);
    _speedComputeTimer.priority(254);

    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
}

void FlywheelMotor::startMotor(){
    _f.flush();
    _speed = 0;
    _motor_enc.write(0);
    _prevAngle = 0;
    _currentAngle = 0;
    _speed = 0;
    _outputSum = 0;
    flywheelMotor.computeCommand();
    _speedComputeTimer.begin(computeSpeedTimer, COMPUTE_INTERVAL);
    _speedMeasureTimer.begin(measureSpeedTimer, SPEED_INTERVAL);
}

void FlywheelMotor::stopMotor(){
    _speedMeasureTimer.end();
    _speedComputeTimer.end();

    setMotorSpeed(0, CW);
    delay(50);

    Serial.print("!");
    Serial.print(_Kp, 5);
    Serial.print(", ");
    Serial.print(_Ki, 5);
    Serial.print(", ");
    Serial.println(_Kd, 5);
}

//Return encoder value, not used
double FlywheelMotor::readAngle(){
    return _motor_enc.read()*COUNT_TO_ANGLE;
}

//Calculate motor speed
void FlywheelMotor::measureSpeed(){    
    _prevAngle = _currentAngle;
    _currentAngle = readAngle();


    _speedRaw = ((_currentAngle - _prevAngle) * USEC_TO_SEC) / SPEED_INTERVAL;
    _speed = _f.filterIn(_speedRaw);

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
    double error = _targetSpeed - _speed;
    double output;

    _outputSum += (_KiSamp * error);
    output = _outputSum - _Kp * _speed;

    //Saturate output voltage to +/- 6V
    if(output > 6) output = 6;
    else if(output < -6) output = -6;  

    _speedCommand = output;

    printMotorData();

    double pwmVal = _speedCommand*V_TO_PWM;
    setMotorSpeed(pwmVal);

    // Serial.print("Speed: ");
    // Serial.print(_speedF);
    // Serial.print("\tError: ");
    // Serial.print(error);
    // Serial.print("\t OutputSum: ");
    // Serial.print(_outputSum);
    // Serial.print("\t -Kp*speed: ");
    // Serial.print(-_Kp * _speedF);
    // Serial.print("\t Tension: ");
    // Serial.println(output);
}


// To measure step reponse
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
// Send bike data to Python Interface
void FlywheelMotor::printMotorData(){
    // Data format:
    // #Bike_Angle, Bike_Angle_Raw, Bike_AngVel, Bike_AngVel_Raw, FW_Angle, FW_Angle_Raw
    //   FW_Speed, FW_Speed_Raw, FW_Target_Speed, FW_Target_Accel, FW_Command, Time
    if(!_printTextData){
        Serial.print("#");
        Serial.print(mainController.getAngle(), 5);
        Serial.print(", ");
        Serial.print(mainController.getAngleRaw(), 5);
        Serial.print(", ");
        Serial.print(mainController.getAngularVel());
        Serial.print(", ");
        Serial.print(mainController.getAngularVelRaw());
        Serial.print(", ");
        Serial.print(_currentAngle);
        Serial.print(", ");
        Serial.print(_currentAngleRaw);
        Serial.print(", ");
        Serial.print(_speed);
        Serial.print(", ");
        Serial.print(_speedRaw);
        Serial.print(", ");
        Serial.print(_targetSpeed);
        Serial.print(", ");
        Serial.print(mainController.getTargetAccel());
        Serial.print(", ");
        Serial.print(_speedCommand);
        Serial.print(", ");
        Serial.println(millis());
    }
    else{
        Serial.print("Time:");
        Serial.print(millis());

        Serial.print("\tBike Angle:");  //deg
        Serial.print(mainController.getAngle(), 5);

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

double FlywheelMotor::getAngleRaw(){return _currentAngleRaw;}

//Return current motor speed in deg/sec
double FlywheelMotor::getSpeed(){ return _speed;}

double FlywheelMotor::getSpeedRaw(){ return _speedRaw;}

//Return current motor speed in rpm
double FlywheelMotor::getSpeedRPM(){ return _speed/6;}

double FlywheelMotor::getTargetSpeed(){ return _targetSpeed;}

double FlywheelMotor::getKp(){return _Kp;}

double FlywheelMotor::getKi(){return _Ki;}

double FlywheelMotor::getKd(){return _Kd;}


void FlywheelMotor::setTargetSpeed(double targetSpeed){ _targetSpeed = targetSpeed;}

void FlywheelMotor::setKp(double Kp){_Kp = Kp;}

void FlywheelMotor::setKi(double Ki){
    _Ki = Ki;
    _KiSamp = _Ki*COMPUTE_INTERVAL/1000000;
}

void FlywheelMotor::setKd(double Kd){ _Kd = Kd;}