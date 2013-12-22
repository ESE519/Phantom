#include "MotorControl.h"


MotorControl::MotorControl(DigitalOut *MotorDirection, PwmOut *MotorEnable)
{
    motorDirection = MotorDirection;
    motorEnable = MotorEnable;
    motorDirection->write(0);
    motorEnable->period_us(1000);
    motorEnable->pulsewidth_ms(0);
    _speed = 0;
    _direction = 0;
}


void MotorControl::motorSpeedControl(float Speed)        // speed range: 0 - 100
{
    _speed = Speed;
    if(_speed > 100)
    {
        _speed = 100;
    }
    if(_speed <0)
    {
        _speed = 0;
    }
    _speed = (int)(_speed / 100 * 1000);
    motorEnable->pulsewidth_us(_speed);
}

 void MotorControl::motorDirectionControl(bool Direction)
{
    _direction = Direction;
    motorDirection->write(_direction);
}
