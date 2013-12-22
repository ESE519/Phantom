#include "mbed.h"
#include "IMU_MPU6050.h"
#include "MPU6050.h"


DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);

MPU6050 imu_mpu6050(p28, p27);      // p28 sda, p27 scl, define the IMU connection 

Ticker DataUpdateIMU;       // define the ticker to update the data from IMU
float updateTimeIMU = 0.01;         // unit is second, update date time
float beta_a = 0.95;        // coefficient for low-pass filter
float beta_g = 0.95;        // coefficient for high-pass filter

IMU_MPU6050 imu_hand(&imu_mpu6050, &DataUpdateIMU, updateTimeIMU, beta_a, beta_g);      // define the IMU device



int main() {
    pc.baud(57600);     // set the serial baudrate
    imu_mpu6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);    // set the scale of the accelerometer
    imu_mpu6050.setGyroRange(MPU6050_GYRO_RANGE_250);           // set the scale of the gyro
    float angle_x;
    float angle_y;      // store the angle of x-axis and angle of y-axis
    bool success_flag;  // indicate the iic connection
    
    success_flag = imu_hand.testCommunication();    // if successful, return 1
    while(!success_flag)        // wait until connection
    {
        myled = 1;
    }   
    
    while(1) {      
        imu_hand.getAngleXY(&angle_x, &angle_y);    // get angle_x and angle_y
        pc.printf("angle_x: %f\n\r",angle_x);   
        pc.printf("angle_y: %f\n\r",angle_y);       // print out the results 
    }
}