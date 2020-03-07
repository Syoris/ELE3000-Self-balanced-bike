#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Encoder.h"
#include <PID_v1.h>

//Define
#define CW true
#define CCW false

#define DEBUG_MOTOR 0

#define ENC_PIN_1 12
#define ENC_PIN_2 11

#define MOTOR_PIN_1 4
#define MOTOR_PIN_2 3

#define COUNT_PER_TURN 44
#define GEARBOX_RATIO 23.1
#define COUNT_TO_ANGLE 360/(GEARBOX_RATIO*COUNT_PER_TURN)
#define SPEED_INTERVAL 300000 //500ms Interval to measure speed (in uS)
#define USEC_TO_SEC 1000000

class FlywheelMotor{
    private:
        Encoder _motor_enc;
        IntervalTimer _speedTimer;

        double _speed;
        double _prevAngle;
        double _currentAngle;

        //PID variables
        PID _speedPID;
        double _targetSpeed;
        double _speedCommand;
        double _Kp, _Kd, _Ki;

        //Pinout
        int _pin1 = MOTOR_PIN_1;
        int _pin2 = MOTOR_PIN_2;
        
    public:
        FlywheelMotor();

        void initMotor();

        double readAngle();
        void measureSpeed();

        double getSpeed();
        double getSpeedRPM();
        double getAngle();

        void setMotorSpeed(int speed, bool dir);
        void setMotorSpeedPID(double targetSpeed);
        void brakeMotor();
};

extern FlywheelMotor flywheelMotor;

#endif