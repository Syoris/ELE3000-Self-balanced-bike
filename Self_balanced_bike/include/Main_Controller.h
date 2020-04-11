#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <Arduino.h>
#include <PID_C.h>
#include <IMU.h>
#include <Motor_Controller.h>
#include <filters.h>

#define COMPUTE_INTERVAL_ANGLE 15000.0 //Interval to compute PID for speed control (in uS)
#define ZERO_OFFSET 2.6*DEG_TO_RAD
//#define RAD_TO_DEG 180/M_PI
//#define DEG_TO_RAD M_PI/180


class MainController{
    private:
        PID _anglePID;

        double _currentAngle;
        double _prevAngle;
        double _targetAngle;
        double _accelOutput;    //Target speed of flywheel
        double _prevComputeTime;

        double _Kp, _Ki, _Kd;

        float _ypr[3];          //[yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        double _angVel;
        double _prevAngVelF;
        double _angVelF;

        bool _toStabilize = false;
        bool _imuRdy = false;

        // Filtre
        Filter _f;
    
    public:
        MainController();

        void startController();
        void stopController();

        void updateAngle();
        void computeCommand();
        void measureAngVel();


        //! Interface
        double getAngle();
        double getAngularVel();
        double getAngularVelFiltered();
        double getTargetAngle();
        double getTargetAccel();
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

extern MainController mainController;

#endif