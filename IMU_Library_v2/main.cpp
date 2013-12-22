#include "mbed.h"
#include "IMU_MPU6050.h"
#include "MPU6050.h"


DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);

MPU6050 imu_mpu6050(p28, p27);      // p28 sda, p27 scl


Ticker DataUpdateIMU;
float updateTimeIMU = 0.01;         // unit is second
float beta_a = 0.95;
float beta_g = 0.95;

IMU_MPU6050 imu_hand(&imu_mpu6050, &DataUpdateIMU, updateTimeIMU, beta_a, beta_g);



int main() {
    pc.baud(57600);
    imu_mpu6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    imu_mpu6050.setGyroRange(MPU6050_GYRO_RANGE_250);
    float angle_x;
    float angle_y;
    float angle_z;
    
    bool success_flag;
    
    success_flag = imu_hand.testCommunication();
    while(!success_flag)
    {
        myled = 1;
    }
    
    while(1) {
        
        imu_hand.getAngleXYZ(&angle_x, &angle_y, &angle_z);
        pc.printf("angle_x: %f\n\r",angle_x);
        pc.printf("angle_y: %f\n\r",angle_y); 
        pc.printf("angle_z: %f\n\r",angle_z);   
    }
}
