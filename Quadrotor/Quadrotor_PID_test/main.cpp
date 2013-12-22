#include "mbed.h"
#include "IMU_MPU6050.h"
#include "MPU6050.h"
#include "BLDC_Control.h"
#include "PID_BLDC_Control.h"
#include "msg_interface.h"
#include "message_definitions.h"

#define ThrustUp 1
#define ThrustDown 2
#define RollUp 3
#define RollDown 4
#define PitchUp 5
#define PitchDown 6
#define YawClockwise 7
#define YawAntiClockwise 8

DigitalOut myled(LED1);
DigitalOut led2(LED2);
Serial pc(USBTX, USBRX);

// receive command
DigitalOut got_packet(LED3);
DigitalOut rst(p20);    // reset for xBee
Serial xbee(p9,p10);
MessageIface msgIface(&xbee);
MsgMode *msg_mode;      // define the structure for receiving

// front motor
PwmOut motor_front_speed(p21);
BLDC_Control motor_front(&motor_front_speed);
// back motor
PwmOut motor_back_speed(p23);
BLDC_Control motor_back(&motor_back_speed);
// left motor
PwmOut motor_left_speed(p24);
BLDC_Control motor_left(&motor_left_speed);
// right motor
PwmOut motor_right_speed(p22);
BLDC_Control motor_right(&motor_right_speed);

// IMU definition
MPU6050 imu_mpu6050(p28, p27);      // p28 sda, p27 scl
Ticker DataUpdateIMU;
float updateTimeIMU = 0.01;         // unit is second
float beta_a = 0.95;
float beta_g = 0.95;
IMU_MPU6050 imu_quadrotor(&imu_mpu6050, &DataUpdateIMU, updateTimeIMU, beta_a, beta_g);


// PID class
uint8_t ID = 1;
Ticker PIDDataUpdate;
float updateTimePID = 0.01;
float Kp = 1;
float Ki = 0;
float Kd = 0;
float Kd_yaw = 0;
PID_BLDC_Control pid_quadrotor(ID, &imu_quadrotor, &PIDDataUpdate, updateTimePID, Kp, Ki, Kd, Kd_yaw);


int main() {
    float angle_x, angle_y, angle_z;        // for testing
    
    // xbee received data 
    uint8_t *rx_data;
    uint8_t rx_len;
    uint8_t msg_type;    
    uint8_t *msg_data;
    uint8_t mode_value;
    // xbee initialize
    xbee.baud(57600);
    rst = 0;
    myled = 0;
    wait_ms(1);
    rst = 1;
    wait_ms(1);
    myled = 1;
    myled = 0;
    
    pc.baud(57600);
    imu_mpu6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    imu_mpu6050.setGyroRange(MPU6050_GYRO_RANGE_250);
    bool success_flag;
    // definition of motor speed
    float frontCmdSpeed = 50, backCmdSpeed = 50, leftCmdSpeed = 50, rightCmdSpeed = 50;
    float des_roll, des_pitch, des_yaw, des_thrust;
    success_flag = imu_quadrotor.testCommunication();
    
    while(!success_flag)
    {
        myled = 1;
    }
    
    motor_front.motorInit();
    motor_back.motorInit();
    motor_left.motorInit();
    motor_right.motorInit();
    wait(2);
    while(1) {
        led2 = 1;
        mode_value = 0;
        //myled = 1;
        
        msgIface.getBytes();
        //imu_quadrotor.getAngleXYZ(&angle_x, &angle_y, &angle_z);
        //pc.printf("r: %f\n\r",angle_x);
        //pc.printf("p: %f\n\r",angle_y);
        //pc.printf("y: %f\n\r",angle_z);
        // Handle incoming messages
        if (msgIface.peekPacket( &rx_data, &rx_len ) ) {    // check if I received some data   
            //got_packet = !got_packet;   
            msg_type = rx_data[0];          // get the ID
            msg_data = rx_data+1;           // get the message
            pc.printf("ID: %d\n\r",msg_type);
            switch(msg_type)        // check the ID
            {
                case MsgTypeMode:
                    got_packet = !got_packet;
                    msg_mode = (MsgMode*)msg_data;  // receive the data
                    mode_value = msg_mode->mode_recv;        
                break;  
            }
            
            msgIface.dropPacket();  
        }
        //pc.printf("mode: %d\n\r", mode_value);
        
        
        des_roll = 0;
        des_pitch = 0;
        des_yaw = 0;
        des_thrust = 0;  
          
        pid_quadrotor.setAngleGoal(des_roll, des_pitch, des_yaw, des_thrust);
        
        if(pid_quadrotor.updateSpeed(&pc, &frontCmdSpeed, &leftCmdSpeed, &backCmdSpeed, &rightCmdSpeed))
        {
            //motor_front.motorSpeedControl(frontCmdSpeed);
            //motor_back.motorSpeedControl(backCmdSpeed);
            //motor_left.motorSpeedControl(leftCmdSpeed);
            //motor_right.motorSpeedControl(rightCmdSpeed);
        }
        pc.printf("front: %f\n\r", frontCmdSpeed);
        pc.printf("back: %f\n\r", backCmdSpeed);
        pc.printf("left: %f\n\r", leftCmdSpeed);
        pc.printf("right: %f\n\r", rightCmdSpeed);
        
        /*
        if(frontCmdSpeed >= 100)
        {
            frontCmdSpeed = 0;
        }
        if(backCmdSpeed >= 100)
        {
            backCmdSpeed = 0;
        }
        if(leftCmdSpeed >= 100)
        {
            leftCmdSpeed = 0;
        }
        if(rightCmdSpeed >= 100)
        {
            rightCmdSpeed = 0;
        }
        motor_front.motorSpeedControl(frontCmdSpeed);
        motor_back.motorSpeedControl(backCmdSpeed);
        motor_left.motorSpeedControl(leftCmdSpeed);
        motor_right.motorSpeedControl(rightCmdSpeed);
        frontCmdSpeed += 10;
        backCmdSpeed += 10;
        leftCmdSpeed += 10;
        rightCmdSpeed += 10; 
        */   
        
           
    }
}
