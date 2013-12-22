#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "mbed.h"

class MotorControl
{
private:
    float _speed;
    bool _direction;
    DigitalOut *motorDirection;
    PwmOut *motorEnable;


public:
    MotorControl(DigitalOut *MotorDirection, PwmOut *MotorEnable);
    void motorSpeedControl(float Speed);
    void motorDirectionControl(bool Direction);
    
};

#endif