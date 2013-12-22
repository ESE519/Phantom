#ifndef _IMU_MPU6050_H_
#define _IMU_MPU6050_H_

#include "mbed.h"
#include "MPU6050.h"
#include <stdio.h>
#include <math.h>

using namespace mbed;

class IMU_MPU6050       // version 2 with yaw angle
{
private:
    MPU6050 *imu;
    Ticker *DataUpdate;
    bool flag_update_data;
    float updateTime;   // data update time
    float beta_a;       // coefficient of low-pass filter for accelerometer
    float beta_g;       // coefficient of high-pass filter for gyro
    bool communication_success;     // flag for successful communication
    int imu_accelero_raw[3];        // raw data from accelerometer
    int imu_gyro_raw[3];        // raw data from gyro
    float phi_previous_y;  
    float phi_y;
    float phi_previous_hat_y;
    float phi_hat_y;
    float theta_previous_hat_y;
    float theta_hat_y;
    float phi_previous_x;  
    float phi_x;
    float phi_previous_hat_x;
    float phi_hat_x;
    float theta_previous_hat_x;
    float theta_hat_x;
    float phi_previous_z;
    float phi_z;
    float phi_previous_hat_z;
    float phi_hat_z;
    float angle_y;        // angle for y-axis
    float angle_x;        // angle for x-axis
    float angle_z;
    
    
public:
    IMU_MPU6050(MPU6050 *IMU, Ticker *DATAUPDATE, float UPDATETIME, float BETA_A, float BETA_G);
    float getAngleXYZ(float *angle_x_axis, float *angle_y_axis, float *angle_z_axis);
    bool testCommunication(void); 
    void UpdateData(void);   
};

#endif