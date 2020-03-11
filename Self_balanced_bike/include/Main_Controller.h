#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <PID_v1.h>


class MainController{
    private:
        PID _anglePID;
        IntervalTimer _computeTimer;

        double _currentAngle;
        double _targetAngle;
        double _speedOutput;    //Target speed of flywheel

        double _Kp;
        double _Ki;
        double _Kd;

    
    public:
        MainController();

        void readAngle();

        double getAngle();
        double getTargetAngle();

        void setTargetAngle();

};

#endif