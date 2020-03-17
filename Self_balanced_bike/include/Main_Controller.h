#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>
#include <IMU.h>

#define COMPUTE_INTERVAL_ANGLE 1000 //Interval to compute PID for speed control (in uS)


class MainController{
    private:
        PID _anglePID;
        IntervalTimer _computeTimer;

        double _currentAngle;
        double _targetAngle;
        double _speedOutput;    //Target speed of flywheel

        double _Kp, _Ki, _Kd;
        
        bool _imuRdy = false;
    
    public:
        MainController();

        void startController();
        void stopController();

        void readAngle();
        void computeCommand();


        //! Interface
        double getAngle();
        double getTargetAngle();
        double getKp();
        double getKi();
        double getKd();

        void setTargetAngle(double newAngle);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
        void setPID();
        void setPID(double Kp, double Ki, double Kd);

};

#endif