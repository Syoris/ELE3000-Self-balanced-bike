#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Encoder.h"

//Define
#define CW true
#define CCW false

#define ENC_PIN_1 12
#define ENC_PIN_2 11

#define MOTOR_PIN_1 1
#define MOTOR_PIN_2 2

#define COUNT_TO_ANGLE 360/(21*11)
#define SPEED_INTERVAL 300000 //Interval to measure speed (in uS)
#define USEC_TO_SEC 1000000

class FlywheelMotor{
    private:
        Encoder _motor_enc;
        IntervalTimer _speedTimer;

        double _speed;
        double _prevAngle;
        double _currentAngle;

        int _pin1 = MOTOR_PIN_1;
        int _pin2 = MOTOR_PIN_2;
        
    public:
        double readAngle();
        FlywheelMotor();
        void measureSpeed();

        double getSpeed();
        double getAngle();

        void setMotorSpeed(int speed, bool dir);
        void brakeMotor();
};

extern FlywheelMotor flywheelMotor;

#endif