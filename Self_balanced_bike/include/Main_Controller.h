#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <Arduino.h>
#include <IMU.h>
#include <Motor_Controller.h>
#include <filters.h>

#define COMPUTE_INTERVAL_ANGLE 10.0 //Interval to compute PID for speed control (in mS)
#define SPEED_MEASURE_INTERVAL 5.0 //Interval to compute PID for speed control (in mS)
#define ANGLE_MEASURE_INTERVAL 1.0 //Interval to compute PID for speed control (in mS)
#define SEND_DATA_INTERVAL 10.0 //Interval to compute PID for speed control (in mS)

#define ZERO_OFFSET 0.0*DEG_TO_RAD
#define DEAD_ZONE   0.0*DEG_TO_RAD
#define DEAD_TIME 0.1
#define SPD_UP_THRES 6500
#define SPD_DWN_TRHES 150
#define DANGER_ANGLE_TARGET 1*DEG_TO_RAD

class MainController{
    private:
        // Bike angle
        float _zeroOffset;
        double _currentAngle;
        double _currentAngleRaw;
        double _prevAngle;
        double _targetAngle;
        float _ypr[3];          //[yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        // Bike Angular velocity
        double _angVel;     
        double _angVelRaw;
        double _prevAngVel;

        unsigned long _prevComputeTime;
        unsigned long _prevSpeedTime;
        unsigned long _prevAngleTime;
        unsigned long _prevPrintTime;
        
        // PID variables
        double _accelOutput;    //Target speed of flywheel
        double _Kp, _Ki, _Kd;
        double _outputSum;
        double _deadZone;
        unsigned long _deadTime;
        bool _inDeadZone;
        bool _inDanger;

        bool _toStabilize = false;
        bool _imuRdy = false;

        // Filtre
        Filter _speedFilter;
        Filter _angleFilter;
    
    public:
        MainController();

        void init();

        void startController();
        void stopController();

        void updateAngle();
        void measureSpeed(double timeInt);
        void computeCommand(double timeInt);
        void stabilise();

        //! Interface
        double getAngle();
        double getAngleRaw();
        double getAngularVel();
        double getAngularVelRaw();
        double getTargetAngle();
        double getTargetAccel();
        float getZeroOffset();
        double getDeadZone();

        double getKp();
        double getKi();
        double getKd();

        void setTargetAngle(double newAngle);
        void setZeroOffset(float newOffset);
        void setDeadZone(double newDeadZone);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
};

extern MainController mainController;

#endif