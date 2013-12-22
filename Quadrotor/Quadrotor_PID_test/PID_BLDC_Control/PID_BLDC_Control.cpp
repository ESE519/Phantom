#include "PID_BLDC_Control.h"

PID_BLDC_Control::PID_BLDC_Control(uint8_t ID, IMU_MPU6050 *IMU, Ticker *DATAUPDATE, float UPDATETIME, float KP, float KI, float KD, float KD_YAW)
{
    id = ID;
    Kp = KP;
    Kd = KD;
    Ki = KI;
    Kd_yaw = KD_YAW;
    imu = IMU;
    DataUpdate = DATAUPDATE;
    updateTime = UPDATETIME;
    previousYaw = 0;
    previousRoll = 0;
    previousPitch = 0;
    desiredYaw = 0;
    desiredRoll = 0;
    desiredPitch = 0;
    desiredThrust = 0;
    currentYaw = 0;
    currentRoll = 0;
    currentPitch = 0;
    previous_error_yaw = 0;
    previous_error_roll = 0;
    previous_error_pitch = 0;
    updateTime = UPDATETIME;
    //QEI encoder
    DataUpdate->attach(this,&PID_BLDC_Control::UpdateData,updateTime);    
}

void PID_BLDC_Control::UpdateData(void)
{
    float angle_roll, angle_pitch, angle_yaw;
    imu->getAngleXYZ(&angle_roll, &angle_pitch, &angle_yaw);
    //printf("angle_roll: %f\n\r",angle_roll);
    //printf("angle_pitch: %f\n\r",angle_pitch);
    //printf("angle_yaw: %f\n\r",angle_yaw);
    currentYaw = angle_yaw;
    currentPitch = angle_pitch;
    currentRoll = angle_roll;
    flag_update_PID = 1;
}

void PID_BLDC_Control::setAngleGoal(float des_roll, float des_pitch, float des_yaw, float des_thrust)
{
    desiredRoll = des_roll;
    desiredPitch = des_pitch;
    desiredYaw = des_yaw;
    desiredThrust = des_thrust;
}

void PID_BLDC_Control::return_currentAngle(float *cur_roll, float *cur_pitch, float *cur_yaw)
{
    *cur_roll = currentRoll;
    *cur_pitch = currentPitch;
    *cur_yaw = currentYaw;
}

int PID_BLDC_Control::updateSpeed(Serial *serial, float *cmdSpeed0, float *cmdSpeed1, float *cmdSpeed2, float *cmdSpeed3)
{
    float d_omega_x, d_omega_y, d_omega_z, cmdSpeedBase;
    float w_roll, w_pitch, w_yaw;
    float output_0, output_1, output_2, output_3; 
    if(flag_update_PID == 0)
    {
        return 0;
    }
    else
    {
        if(desiredThrust > 100)
        {
            desiredThrust = 100;
        }
        if(desiredThrust < 0)
        {
            desiredThrust = 0;
        }
        if(desiredRoll > 30)
        {
            desiredRoll = 30;
        }
        if(desiredRoll < -30)
        {
            desiredRoll = -30;
        }
        if(desiredPitch > 30)
        {
            desiredPitch = 30;
        }
        if(desiredPitch < -30)
        {
            desiredPitch = -30;
        }
        if(desiredYaw > 180)
        {
            desiredYaw = 180;
        }
        if(desiredYaw < -180)
        {
            desiredYaw = -180;
        }
        
        serial->printf("roll: %f\n\r", currentRoll);
        serial->printf("pitch: %f\n\r", currentPitch);
        serial->printf("yaw: %f\n\r", currentYaw);
        
        w_roll = (currentRoll - previousRoll)/updateTime;
        w_pitch = (currentPitch - previousPitch)/updateTime;
        w_yaw = (currentYaw - previousYaw)/updateTime;
        
        previousRoll = currentRoll;
        previousPitch = currentPitch;
        previousYaw = currentYaw;
        
        cmdSpeedBase = desiredThrust;
        
        d_omega_x = Kp * (desiredRoll - currentRoll) + Kd * (-w_roll);
        d_omega_y = Kp * (desiredPitch - currentPitch) + Kd * (-w_pitch);
        d_omega_z = Kp * (desiredYaw - currentYaw) + Kd_yaw * (-w_yaw);
        
        //d_omega_z = 0;
        
        serial->printf("d_x: %f\n\r", d_omega_x);
        serial->printf("d_y: %f\n\r", d_omega_y);
        serial->printf("d_z: %f\n\r", d_omega_z);
        
        output_0 = cmdSpeedBase - d_omega_y + d_omega_z;
        output_1 = cmdSpeedBase + d_omega_x - d_omega_z;
        output_2 = cmdSpeedBase + d_omega_y + d_omega_z;
        output_3 = cmdSpeedBase - d_omega_x - d_omega_z;
        if(output_0 > 100)
        {
            output_0 = 100;
        }
        if(output_0 < 0)
        {
            output_0 = 0;
        }
        if(output_1 > 100)
        {
            output_1 = 100;
        }
        if(output_1 < 0)
        {
            output_1 = 0;
        }
        if(output_2 > 100)
        {
            output_2 = 100;
        }
        if(output_2 < 0)
        {
            output_2 = 0;
        }
        if(output_3 > 100)
        {
            output_3 = 100;
        }
        if(output_3 < 0)
        {
            output_3 = 0;
        }
        
        *cmdSpeed0 = output_0;
        *cmdSpeed1 = output_1;
        *cmdSpeed2 = output_2;
        *cmdSpeed3 = output_3;
        flag_update_PID = 0;
        return 1;
    }
    
}   