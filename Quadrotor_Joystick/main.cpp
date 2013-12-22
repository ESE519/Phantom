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
MsgQuadrotor *msg_quadrotor; // define the structure for receiving quadrotor parameters

// feedback
/*
MsgIMUFeedback msg_imu_feedback;
Ticker send;
void sendPackage(void);
*/
float cR, cP, cY;    // IMU feedback  

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
float updateTimeIMU = 0.003;         // unit is second
float beta_a = 0.98;
float beta_g = 0.98;
IMU_MPU6050 imu_quadrotor(&imu_mpu6050, &DataUpdateIMU, updateTimeIMU, beta_a, beta_g);


// PID class
uint8_t ID = 1;
Ticker PIDDataUpdate;
float updateTimePID = 0.003;
float Kp = 0;
float Ki = 0;
float Kd = 0;
float Kd_yaw = 0;
PID_BLDC_Control pid_quadrotor(ID, &imu_quadrotor, &PIDDataUpdate, updateTimePID, Kp, Ki, Kd, Kd_yaw);

Timer estop;
bool flag_recv;

int main() {
    //float angle_x, angle_y, angle_z;        // for testing
  
    // xbee received data 
    uint8_t *rx_data;
    uint8_t rx_len;
    uint8_t msg_type;    
    uint8_t *msg_data;
    uint8_t mode_value;
    
    //send.attach(&sendPackage,0.1);  // feedback configuration
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
    float frontCmdSpeed = 0, backCmdSpeed = 0, leftCmdSpeed = 0, rightCmdSpeed = 0;
    float des_roll, des_pitch, des_yaw, des_thrust;
    success_flag = imu_quadrotor.testCommunication();
    
    while(!success_flag)
    {
        myled = 1;
    }
    
    flag_recv = 0;
    estop.start();
    estop.reset();
    
    motor_front.motorInit();
    motor_back.motorInit();
    motor_left.motorInit();
    motor_right.motorInit();
    wait(2);
    while(1) {
        led2 = 1;
        //mode_value = 0;
        //des_roll = 0;
        //des_pitch = 0;
        //des_yaw = 0;
        //des_thrust = 0;
        
        msgIface.getBytes();
        
        //IMUFeedback msg_imu_feedback;
        /*
        msg_imu_feedback.id = 0;
        imu_quadrotor.getAngleXYZ(&currentRoll, &currentPitch, &currentYaw);   
        msg_imu_feedback.current_roll = currentRoll;
        msg_imu_feedback.current_pitch = currentPitch; 
        msg_imu_feedback.current_yaw = currentYaw; 
        msg_imu_feedback.current_thrust = 0; 
        */

        
        // Handle incoming messages
        
        if (msgIface.peekPacket( &rx_data, &rx_len ) ) {    // check if I received some data   
            got_packet = !got_packet;   
            msg_type = rx_data[0];          // get the ID
            msg_data = rx_data+1;           // get the message
            //pc.printf("ID: %d\n\r",msg_type);
            switch(msg_type)        // check the ID
            {
                case MsgTypeMode:
                    //got_packet = !got_packet;
                    msg_mode = (MsgMode*)msg_data;  // receive the data
                    pid_quadrotor.Kp = msg_mode->Kp_recv;
                    pid_quadrotor.Ki = msg_mode->Ki_recv;
                    pid_quadrotor.Kd = msg_mode->Kd_recv;
                    pid_quadrotor.Kd_yaw = msg_mode->Kd_yaw_recv;
                    pc.printf("Kp: %f\n\r", pid_quadrotor.Kp);
                    pc.printf("Ki: %f\n\r", pid_quadrotor.Ki);
                    pc.printf("Kd: %f\n\r", pid_quadrotor.Kd);
                    pc.printf("Kd_yaw: %f\n\r", pid_quadrotor.Kd_yaw); 
                break;  
                case MsgTypeQuadrotor:
                    //got_packet = !got_packet;
                    msg_quadrotor = (MsgQuadrotor*)msg_data;
                    des_roll = msg_quadrotor->desired_roll_recv;
                    des_pitch = msg_quadrotor->desired_pitch_recv;
                    des_yaw = msg_quadrotor->desired_yaw_recv;
                    des_thrust = msg_quadrotor->desired_thrust_recv;
                    if(des_thrust < 5)
                    {
                        flag_recv = 0;
                    }
                    else
                    {
                        flag_recv = 1;
                    }
                break;
            }
            
            msgIface.dropPacket();  
        }
        
        /*
        if(des_roll > 5)
        {
            myled = !myled;
        }
        */
        
        //pc.printf("roll: %f\n\r", des_roll);
        //pc.printf("pitch: %f\n\r", des_pitch);
        //pc.printf("yaw: %f\n\r", des_yaw);
        //pc.printf("thrust: %f\n\r", des_thrust);
        //pc.printf("P: %f\n\r", Kp);
        //pc.printf("I: %f\n\r", Ki);
        //pc.printf("D: %f\n\r", Kd);

        
        //pc.printf("Croll: %f\n\r", currentRoll);
        //pc.printf("Cpitch: %f\n\r", currentPitch);
        //pc.printf("Cyaw: %f\n\r", currentYaw);
        
        //motor_front.motorSpeedControl(10);
        //motor_back.motorSpeedControl(1);
        //motor_left.motorSpeedControl(0);
        //motor_right.motorSpeedControl(5);
        // DEBUG: resist all motion
        
        //des_roll = 0;
        //des_pitch = 0;
        //des_yaw = 0;
        //des_thrust = 0;
        
        flag_recv = 1;
        if(flag_recv) {
        pid_quadrotor.setAngleGoal(des_roll, des_pitch, des_yaw, des_thrust);
        
        if(pid_quadrotor.updateSpeed(&pc, &frontCmdSpeed, &leftCmdSpeed, &backCmdSpeed, &rightCmdSpeed))
        {
            //motor_front.motorSpeedControl(frontCmdSpeed);
            //motor_back.motorSpeedControl(backCmdSpeed);
            //motor_left.motorSpeedControl(leftCmdSpeed);
            //motor_right.motorSpeedControl(rightCmdSpeed);
            //pc.printf("front: %f\n\r", frontCmdSpeed);
            //pc.printf("back: %f\n\r", backCmdSpeed);
            //pc.printf("left: %f\n\r", leftCmdSpeed);
            //pc.printf("right: %f\n\r", rightCmdSpeed);
        }
        
        }
        else {
            motor_front.motorSpeedControl(0);
            motor_back.motorSpeedControl(0);
            motor_left.motorSpeedControl(0);
            motor_right.motorSpeedControl(0);
        }   
        
        //pid_quadrotor.return_currentAngle(&currentRoll, &currentPitch, &currentYaw);       
        

                
        //msgIface.sendNow();          
    }
}
/*
void sendPackage() {
    msg_imu_feedback.id = 0;
    imu_quadrotor.getAngleXYZ(&cR, &cP, &cY);   
    msg_imu_feedback.current_roll = cR;
    msg_imu_feedback.current_pitch = cP; 
    msg_imu_feedback.current_yaw = cY; 
    msg_imu_feedback.current_thrust = 0;
    pc.printf("r: %f\n\r",cR);
    pc.printf("p: %f\n\r",cP);
    pc.printf("y: %f\n\r",cY); 
    msgIface.sendPacket( MsgTypeIMUFeedback, (uint8_t*)&msg_imu_feedback, sizeof(MsgIMUFeedback) ); 
}
*/


