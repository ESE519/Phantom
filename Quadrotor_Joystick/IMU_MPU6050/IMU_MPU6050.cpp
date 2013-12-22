#include "IMU_MPU6050.h"

IMU_MPU6050::IMU_MPU6050(MPU6050 *IMU, Ticker *DATAUPDATE, float UPDATETIME, float BETA_A, float BETA_G)
{
    imu = IMU;
    DataUpdate = DATAUPDATE;
    updateTime = UPDATETIME;
    beta_a = BETA_A;
    beta_g = BETA_G;
    DataUpdate->attach(this,&IMU_MPU6050::UpdateData,updateTime);
    flag_update_data = 0;
    phi_x = 0;
    phi_y = 0;
    phi_z = 0;
    phi_previous_x = 0;
    phi_previous_y = 0;
    phi_previous_z = 0;
    phi_previous_hat_x = 0;
    phi_previous_hat_y = 0;
    phi_previous_hat_z = 0;
    phi_hat_x = 0;
    phi_hat_y = 0;
    phi_hat_z = 0;
    theta_hat_x = 0;
    theta_hat_y = 0;
    theta_previous_hat_x = 0;
    theta_previous_hat_y = 0;
    angle_x = 0;
    angle_y = 0;
    angle_z = 0;
}

void IMU_MPU6050::UpdateData(void)
{
    imu->getAcceleroRaw(imu_accelero_raw);
    imu->getGyroRaw(imu_gyro_raw);
    flag_update_data = 1;
}

bool IMU_MPU6050::testCommunication()
{
    communication_success = imu->testConnection();
    return communication_success;
}

float IMU_MPU6050::getAngleXYZ(float *angle_x_axis, float *angle_y_axis, float *angle_z_axis)
{
    if(flag_update_data)
    {
        phi_y = phi_previous_y + (float)imu_gyro_raw[1]/16384 * 250 * updateTime;
        phi_hat_y = beta_g * phi_previous_hat_y + beta_g * (phi_y - phi_previous_y);
        imu_accelero_raw[0] -= 400;
        imu_accelero_raw[1] += 50; 
        if(imu_accelero_raw[0] > 8192)
        {   
            imu_accelero_raw[0] = 8190;
        }
        if(imu_accelero_raw[0] < -8192)
        {
            imu_accelero_raw[0] = -8192;
        }
        theta_hat_y = beta_a * theta_previous_hat_y + (1-beta_a) * asin(-(float)imu_accelero_raw[0]/16384 * 2)/3.14*180;
        angle_y = phi_hat_y + theta_hat_y;
        phi_previous_y = phi_y;
        phi_previous_hat_y = phi_hat_y;
        theta_previous_hat_y = theta_hat_y;
        *angle_y_axis = angle_y;
        
        phi_x = phi_previous_x + (float)imu_gyro_raw[0]/16384 * 250 * updateTime;
        phi_hat_x = beta_g * phi_previous_hat_x + beta_g * (phi_x - phi_previous_x);
        if(imu_accelero_raw[1] > 8192)
        {   
            imu_accelero_raw[1] = 8190;
        }
        if(imu_accelero_raw[1] < -8192)
        {
            imu_accelero_raw[1] = -8192;
        }
        theta_hat_x = beta_a * theta_previous_hat_x + (1-beta_a) * asin((float)imu_accelero_raw[1]/16384 * 2)/3.14*180;
        angle_x = phi_hat_x + theta_hat_x;
        phi_previous_x = phi_x;
        phi_previous_hat_x = phi_hat_x;
        theta_previous_hat_x = theta_hat_x;
        *angle_x_axis = angle_x;
        
        phi_z = phi_previous_z + (float)imu_gyro_raw[2]/16384 * 250 * updateTime;
        //phi_hat_z = beta_g * phi_previous_hat_z + beta_g * (phi_z - phi_previous_z);
        angle_z = phi_z;
        phi_previous_z = phi_z;
        //phi_previous_hat_z = phi_hat_z;
        *angle_z_axis = angle_z;
                
        flag_update_data = 0;
        
        return 1;
    }
    else
    {
        return 0;
    }
    
}
