#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Encoder.h"
#include <Main_Controller.h>
#include <filters.h>

//Define
#define MAX_SPEED 8000.0 //Max motor speed [deg/sec]
#define MAX_ACCEL 25000.0 //Max motor accell [deg/sec^2]

#define CW true
#define CCW false

#define DEBUG_MOTOR 0

#define ENC_PIN_1 12
#define ENC_PIN_2 11
 
#define MOTOR_PIN_1 4
#define MOTOR_PIN_2 3

#define COUNT_PER_TURN 44.0
#define GEARBOX_RATIO 4.4 //23.1
#define COUNT_TO_ANGLE 360.0/(GEARBOX_RATIO*COUNT_PER_TURN)
#define USEC_TO_SEC 1000000.0
#define V_TO_PWM 256/6
#define SEC_SPEED USEC_TO_SEC/SPEED_INTERVAL 

#define SPEED_INTERVAL 2500.0 //Interval to measure speed (in uS)
#define COMPUTE_INTERVAL 5000.0 //Interval to compute PID for speed control (in uS)

class FlywheelMotor{
    private:
        Encoder _motor_enc;
        IntervalTimer _speedMeasureTimer;
        IntervalTimer _speedComputeTimer;

        double _speed;
        double _speedRaw;
        double _currentAngle;
        double _currentAngleRaw;
        double _prevAngle;

        //PID variables
        double _targetSpeed;
        double _speedCommand;
        double _Kp, _Kd, _Ki;
        double _KiSamp;     //Ki modified by compute time
        double _outputSum;  //Integral

        //Pinout
        int _pin1 = MOTOR_PIN_1;
        int _pin2 = MOTOR_PIN_2;
        
        //Pour options
        bool _printTextData = false;    //For debugging

        //Filtre
        Filter _speedFilter;

    public:
        FlywheelMotor();

        void startMotor();
        void stopMotor();
        void computeCommand();

        double readAngle();
        void measureSpeed();
        void stepReponse(double stepVoltage);

        void brakeMotor();

        //! Interface
        void printMotorData();

        double getSpeed();
        double getSpeedRaw();
        double getSpeedRPM();
        double getAngle();
        double getAngleRaw();
        double getTargetSpeed();
        double getKp();
        double getKi();
        double getKd();

        void setTargetSpeed(double targetSpeed);
        void setMotorSpeed(int speed, bool dir);
        void setMotorSpeed(int speed);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
};

extern FlywheelMotor flywheelMotor;

#endif