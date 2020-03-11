#include "Main_Controller.h"

MainController mainController;

void computeAngleTimer(){

}

MainController::MainController():_anglePID(&_currentAngle, &_speedOutput, &_targetAngle, _Kp, _Ki, _Kd, DIRECT),
                                _computeTimer(){
    _targetAngle = 0;
    _Kp = 0.1;
    _Ki = 0;
    _Kd = 0;

    
}