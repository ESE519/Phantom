#ifndef _PID_BLDC_CONTROL_H_
#define _PID_BLDC_CONTROL_H_

#include "mbed.h"
#include "IMU_MPU6050.h"

using namespace mbed;

class PID_BLDC_Control
{
private:
    uint8_t id;
    Ticker *DataUpdate;
    IMU_MPU6050 *imu;
    float updateTime;
    float Kp;
    float Kd;
    float Ki;
    float Kd_yaw;
    float desiredYaw;
    float desiredRoll;
    float desiredPitch;
    float desiredThrust;
    float currentYaw; 
    float currentRoll;
    float currentPitch;
    float previousYaw;
    float previousRoll;
    float previousPitch;
    float previous_error_yaw;
    float previous_error_roll;
    float previous_error_pitch;
    bool flag_update_PID;
    
public:
    PID_BLDC_Control(uint8_t ID, IMU_MPU6050 *IMU, Ticker *DATAUPDATE, float UPDATETIME, float KP, float KI, float KD, float KD_YAW);
    void UpdateData(void); 
    void setAngleGoal(float des_roll, float des_pitch, float des_yaw, float des_thrust);
    void return_currentAngle(float *cur_roll, float *cur_pitch, float *cur_yaw);
    int updateSpeed(Serial *serial, float *cmdSpeed0, float *cmdSpeed1, float *cmdSpeed2, float *cmdSpeed3);    
};

#endif