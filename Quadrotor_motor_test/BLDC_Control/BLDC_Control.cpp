#include "BLDC_Control.h"


BLDC_Control::BLDC_Control(PwmOut *MotorEnable)
{
    motorEnable = MotorEnable;
    motorEnable->period(0.02);
    motorEnable->pulsewidth(0);
    _speed = 0;
}


void BLDC_Control::motorSpeedControl(float Speed)        // speed range: 0 - 100
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
    _speed = (_speed / 100 + 1)/1000;
    motorEnable->pulsewidth(_speed);
}

void BLDC_Control::motorInit(void)
{
    motorEnable->pulsewidth(0.0009);
    //wait(0.1);
    motorEnable->pulsewidth(0.00095);
    //wait(0.1);
}

