#include "mbed.h"
#include "BLDC_Control.h"

// front motor
PwmOut motor_front_speed(p21);
BLDC_Control motor_front(&motor_front_speed);
// back motor
PwmOut motor_back_speed(p22);
BLDC_Control motor_back(&motor_back_speed);
// left motor
PwmOut motor_left_speed(p23);
BLDC_Control motor_left(&motor_left_speed);
// right motor
PwmOut motor_right_speed(p24);
BLDC_Control motor_right(&motor_right_speed);

PwmOut test(p25);


DigitalOut myled(LED1);

int main() {
    float frontCmdSpeed = 50, backCmdSpeed = 50, leftCmdSpeed = 50, rightCmdSpeed = 50;
    float testSpeed;
    test.period(0.02);
    //test.pulsewidth(0.0009);
    //wait(0.1);
    //test.pulsewidth(0.00095);
    //wait(0.1);
    //test.pulsewidth(0.002);
    //motor_front_speed.pulsewidth(0.0009);
    //wait(0.1);
    //motor_front_speed.pulsewidth(0.00095);
    //wait(0.1);
    motor_front.motorInit();
    motor_back.motorInit();
    motor_left.motorInit();
    motor_right.motorInit();
    wait(2);
    while(1) {
       
        myled = 1;
        
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
        
        //testSpeed += 0.00005;
        //test.pulsewidth(testSpeed);
        //if(testSpeed > 0.003)
        //{
        //    testSpeed = 0.001;
        //}
        
        //testSpeed = 0.003;
        //test.pulsewidth(testSpeed);
        
        frontCmdSpeed += 10;
        backCmdSpeed += 10;
        leftCmdSpeed += 10;
        rightCmdSpeed += 10;  
    }
}
