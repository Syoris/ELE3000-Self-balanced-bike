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
#define USEC_TO_SEC 1000000

#define SPEED_INTERVAL 300000 //300ms Interval to measure speed (in uS)
#define COMPUTE_INTERVAL 500000 //Interval to compute PID for speed control (in uS)

class FlywheelMotor{
    private:
        Encoder _motor_enc;
        IntervalTimer _speedMeasureTimer;
        IntervalTimer _speedComputeTimer;

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

        void startMotor();
        void stopMotor();

        double readAngle();
        void measureSpeed();

        void computeCommand();
        
        void brakeMotor();

        //! Interface
        double getSpeed();
        double getSpeedRPM();
        double getAngle();

        void setTargetSpeed(double targetSpeed);
        void setMotorSpeed(int speed, bool dir);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
        void setPID();
        void setPID(double Kp, double Ki, double Kd);
};

extern FlywheelMotor flywheelMotor;

#endif