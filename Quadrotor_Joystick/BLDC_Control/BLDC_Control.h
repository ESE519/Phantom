#ifndef _BLDC_CONTROL_H_
#define _BLDC_CONTROL_H_

#include "mbed.h"

using namespace mbed;

class BLDC_Control
{
private:
    float _speed;
    PwmOut *motorEnable;


public:
    BLDC_Control(PwmOut *MotorEnable);
    void motorSpeedControl(float Speed);
    void motorInit(void);
    
};

#endif