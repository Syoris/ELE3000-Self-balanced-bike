#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <Arduino.h>
#include <PID_C.h>
#include <IMU.h>
#include <Motor_Controller.h>

#define COMPUTE_INTERVAL_ANGLE 5000 //Interval to compute PID for speed control (in uS)


class MainController{
    private:
        PID _anglePID;

        double _currentAngle;
        double _targetAngle;
        double _accelOutput;    //Target speed of flywheel

        double _Kp, _Ki, _Kd;
        float _ypr[3];          //[yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        int32_t _gyro[3];         //Angular speed

        bool _toStabilize = false;
        bool _imuRdy = false;
    
    public:
        MainController();

        void startController();
        void stopController();

        void updateAngle();
        void computeCommand();


        //! Interface
        double getAngle();
        int32_t getAngularSpeed();
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

extern MainController mainController;

#endif