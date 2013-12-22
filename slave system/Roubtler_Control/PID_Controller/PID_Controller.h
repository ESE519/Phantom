#ifndef TABLET_MSG_H
#define TABLET_MSG_H

#include <stdint.h>
#include "debugger.h"
#include "QEI.h"
#include "mbed.h"

using namespace mbed;

class PIDController { 
public:
    PIDController(uint8_t id, QEI *ENCODER, Ticker *DATAUpdate, float updateTIME, float KP, float KI, float KD);
    void UpdateData(void); 
    void setSpeedGoal(int8_t goalSpeed);
    void setPositionGoal(int goalPosition);
    int return_currentPosition(void);
    float return_currentSpeed(void);
    int updateSpeed( Debugger* debug_iface, int8_t *cmdSpeed );
    int update(Serial *pc, int *cmd);
    
private:
    uint8_t _id;
    QEI *encoder;
    Ticker *DataUpdate;
    DigitalOut *led2;
    float updateTime;
    float Kp;
    float Kd;
    float Ki;
    int8_t setSpeed;
    int setPosition;
    int currentPosition;
    float previous_error;
    bool flag_update_PID;
    int currentPosition_previous;
    float integral;
    float currentSpeed;  
    
    // speed low-pass
    float previousSpeed;
    
    // Derivative low-pass
    float prev_derivative;
    float alpha;  
};

#endif