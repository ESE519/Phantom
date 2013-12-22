#include "QEI.h"
#include "mbed.h"
#include "PID_Controller.h"
#include "debugger.h"
#include "MotorControl.h"
#include "msg_interface.h"
#include "message_definitions.h"

#define ID_PID_LEFT 1
#define ID_PID_RIGHT 2
#define ID_PID_FRONT 3
#define ID_PID_BACK 4

#define IDLE 0

#define FORWARD 10
#define BACKWARD 20

#define LEFTWARD 39
#define LEFT_TURN_30 31
#define LEFT_TURN_60 32
#define LEFT_CLOCK 33

#define RIGHTWARD 49
#define RIGHT_TURN_30 41
#define RIGHT_TURN_60 42
#define RIGHT_CLOCK 43

#define NORTHESAST 50
#define SOUTHEAST 60
#define SOUTHWEST 70
#define NORTHWEST 80

#define CANNON_LEFT 101
#define CANNON_RIGHT 102
#define CANNON_UP 103
#define CANNON_DOWN 104
#define SHOOT 105

DigitalOut got_packet(LED3);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led4(LED4);
DigitalOut rst(p20);    // reset for xBee



Serial pc (USBTX,USBRX); 
Serial xbee(p9,p10);

Ticker DataUpdate_left;
Ticker DataUpdate_right;
Ticker DataUpdate_front;
Ticker DataUpdate_back;
 
float updateTime_left = 0.01;              // update time is 100ms
float updateTime_right = 0.01;
float updateTime_front = 0.01;
float updateTime_back = 0.01;  

int setPosition = 0;
int currentPosition = 0;
float Kp_left=0,Ki_left=0,Kd_left=0;
float Kp_right=0,Ki_right=0,Kd_right=0;
float Kp_front=0,Ki_front=0,Kd_front=0;
float Kp_back=0,Ki_back=0,Kd_back=0;




QEI encoder_left(p7, p8, NC, 64, QEI::X2_ENCODING);
QEI encoder_right(p5, p6, NC, 64, QEI::X2_ENCODING);
QEI encoder_front(p11,p12, NC, 64, QEI::X2_ENCODING); 
QEI encoder_back(p13,p14, NC, 64, QEI::X2_ENCODING); 

MessageIface debugIface( &pc );
MessageIface msgIface(&xbee);
Debugger debugger( &debugIface );
MsgMode *msg_mode;      // define the structure for receiving

// front motor
PwmOut motor_front_enable(p21);
DigitalOut motor_front_direction(p27);
MotorControl motor_front(&motor_front_direction, &motor_front_enable);
// back motor
PwmOut motor_back_enable(p22);
DigitalOut motor_back_direction(p28);
MotorControl motor_back(&motor_back_direction, &motor_back_enable);
// left motor
PwmOut motor_left_enable(p23);
DigitalOut motor_left_direction(p29);
MotorControl motor_left(&motor_left_direction, &motor_left_enable);
// right motor
PwmOut motor_right_enable(p24);
DigitalOut motor_right_direction(p30);
MotorControl motor_right(&motor_right_direction, &motor_right_enable);

// horizontal motor cannon
PwmOut motor_horizontal_enable(p25);
DigitalOut motor_horizontal_direction(p17);
MotorControl motor_horizontal(&motor_horizontal_direction, &motor_horizontal_enable);

// vertical motor cannon
PwmOut motor_vertical_enable(p26);
DigitalOut motor_vertical_direction(p18);
MotorControl motor_vertical(&motor_vertical_direction, &motor_vertical_enable);

// shoot
DigitalOut shoot(p19);

Timer estop;
bool flag_recv;

int main() {

    
    // xbee received data 
    uint8_t *rx_data;
    uint8_t rx_len;
    uint8_t msg_type;    
    uint8_t *msg_data;
    uint8_t mode_value;
    
    signed char front_cmd_speed, back_cmd_speed, left_cmd_speed, right_cmd_speed;

    
    PIDController pid_front (ID_PID_FRONT, &encoder_front, &DataUpdate_front, updateTime_front, Kp_front,Ki_front,Kd_front);
    PIDController pid_back (ID_PID_BACK, &encoder_back, &DataUpdate_back, updateTime_back, Kp_back,Ki_back,Kd_back);  
    PIDController pid_left (ID_PID_LEFT, &encoder_left, &DataUpdate_left, updateTime_left, Kp_left, Ki_left, Kd_left);
    PIDController pid_right (ID_PID_RIGHT, &encoder_right, &DataUpdate_right, updateTime_right, Kp_right, Ki_right, Kd_right); 
    encoder_front.reset();
    encoder_back.reset();
    encoder_left.reset();
    encoder_right.reset();
    
    pc.baud(115200);
    
    xbee.baud(57600);
    rst = 0;
    led1 = 0;
    wait_ms(1);
    rst = 1;
    wait_ms(1);
    led1 = 1;
    led1 = 0;

    flag_recv = 0;
    
    estop.start(); 
    estop.reset(); 

    while(1) {
        //led1 = 1;
        //mode_value = 0;
        msgIface.getBytes();
        // Handle incoming messages
        if (msgIface.peekPacket( &rx_data, &rx_len ) ) {    // check if I received some data
            flag_recv = 1;
            led4 = 0;
            got_packet = !got_packet;        
            msg_type = rx_data[0];          // get the ID
            msg_data = rx_data+1;           // get the message
            estop.stop(); 
            estop.reset(); 
            estop.start(); 
            switch(msg_type)        // check the ID
            {
                
                case MsgTypeMode:
                    msg_mode = (MsgMode*)msg_data;  // receive the data
                    mode_value = msg_mode->mode_recv;
                    //pc.printf("%c\n\r",mode_value);
                    //MsgModeFeedback msg_motor_feedback;
                    //msg_motor_feedback.id = 0;
                    //msg_motor_feedback.wheel_cmdr = msg_motor->right_wheel;
                    //msg_motor_feedback.wheel_cmdl = msg_motor->left_wheel; 
                    //msg_motor_feedback.wheel_speedl = pid_left.return_currentSpeed(); 
                    //msg_motor_feedback.wheel_speedr = pid_right.return_currentSpeed(); 
                    //debugIface.sendPacket( MsgTypeMotorFeedback, (uint8_t*)&msg_motor_feedback, sizeof(MsgMotorFeedback) );        
                break;
               
            }
            
            msgIface.dropPacket();  
        }
        pc.printf("mode: %d\n\r", mode_value);
        
        if(flag_recv) {
        
        if (mode_value == IDLE)
        {
            //led4 = !led4;
            //led2 = 0;
            //led1 = 0;
            motor_horizontal.motorSpeedControl(0);  
            motor_horizontal.motorSpeedControl(0);
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(0);
            pid_right.setSpeedGoal(0);
            shoot = 0;
            //wait(0.5);
        }
        
        if(mode_value == FORWARD)
        {   
            //led2 = !led2;
            //led4 = 0;
            //led1 = 0;
            motor_horizontal.motorSpeedControl(0);  
            motor_horizontal.motorSpeedControl(0);
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(60);
            pid_right.setSpeedGoal(60);
            motor_front.motorDirectionControl(0);
            motor_back.motorDirectionControl(0);
            motor_left.motorDirectionControl(1);
            motor_right.motorDirectionControl(0);
            shoot = 0;//wait(0.5);
        }
        
        if(mode_value == BACKWARD)
        {
            motor_horizontal.motorSpeedControl(0);  
            motor_vertical.motorSpeedControl(0);
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(60);
            pid_right.setSpeedGoal(60);
            motor_front.motorDirectionControl(0);
            motor_back.motorDirectionControl(0);
            motor_left.motorDirectionControl(0);
            motor_right.motorDirectionControl(1);
            shoot = 0;    
        }
        //Left series motions
        if(mode_value == LEFTWARD)
        {
            //led1 = !led1;
            //led2 = 0;
            //led4 = 0;
            motor_horizontal.motorSpeedControl(0);  
            motor_vertical.motorSpeedControl(0);
            pid_front.setSpeedGoal(40);
            pid_back.setSpeedGoal(40);
            pid_left.setSpeedGoal(0);
            pid_right.setSpeedGoal(0);
            motor_front.motorDirectionControl(1);
            motor_back.motorDirectionControl(0);
            motor_left.motorDirectionControl(0);
            motor_right.motorDirectionControl(0);
            shoot = 0;
        }
        
        if(mode_value == LEFT_TURN_30)
        {   
            //led2 = !led2;
            //led4 = 0;
            //led1 = 0;
            motor_horizontal.motorSpeedControl(0);  
            motor_horizontal.motorSpeedControl(0);
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(30);
            pid_right.setSpeedGoal(50);
            motor_front.motorDirectionControl(0);
            motor_back.motorDirectionControl(0);
            motor_left.motorDirectionControl(1);
            motor_right.motorDirectionControl(0);
            shoot = 0;//wait(0.5);
        }
        
        if(mode_value == LEFT_TURN_60)
        {   
            //led2 = !led2;
            //led4 = 0;
            //led1 = 0;
            motor_horizontal.motorSpeedControl(0);  
            motor_horizontal.motorSpeedControl(0);
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(30);
            pid_right.setSpeedGoal(70);
            motor_front.motorDirectionControl(0);
            motor_back.motorDirectionControl(0);
            motor_left.motorDirectionControl(1);
            motor_right.motorDirectionControl(0);
            shoot = 0;//wait(0.5);
        }
        
                
        if(mode_value == LEFT_CLOCK)
        {
            motor_horizontal.motorSpeedControl(0);  
            motor_vertical.motorSpeedControl(0);
            pid_front.setSpeedGoal(30);
            pid_back.setSpeedGoal(30);
            pid_left.setSpeedGoal(30);
            pid_right.setSpeedGoal(30);
            
            motor_front.motorDirectionControl(0);
            motor_back.motorDirectionControl(0);
            motor_left.motorDirectionControl(0);
            motor_right.motorDirectionControl(0);            
            shoot = 0;
        }
        //Right series motions
        if(mode_value == RIGHTWARD)
        {
            //led1 = !led1;
            //led2 = 0;
            //led4 = 0;
            motor_horizontal.motorSpeedControl(0);  
            motor_vertical.motorSpeedControl(0);
            pid_front.setSpeedGoal(40);
            pid_back.setSpeedGoal(40);
            pid_left.setSpeedGoal(0);
            pid_right.setSpeedGoal(0);
            motor_front.motorDirectionControl(0);
            motor_back.motorDirectionControl(1);
            motor_left.motorDirectionControl(0);
            motor_right.motorDirectionControl(0);
            shoot = 0;
        }
        
        if(mode_value == RIGHT_TURN_30)
        {   
            //led2 = !led2;
            //led4 = 0;
            //led1 = 0;
            motor_horizontal.motorSpeedControl(0);  
            motor_horizontal.motorSpeedControl(0);
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(50);
            pid_right.setSpeedGoal(30);
            motor_front.motorDirectionControl(0);
            motor_back.motorDirectionControl(0);
            motor_left.motorDirectionControl(1);
            motor_right.motorDirectionControl(0);
            shoot = 0;//wait(0.5);
        }
        
        if(mode_value == RIGHT_TURN_60)
        {   
            //led2 = !led2;
            //led4 = 0;
            //led1 = 0;
            motor_horizontal.motorSpeedControl(0);  
            motor_horizontal.motorSpeedControl(0);
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(70);
            pid_right.setSpeedGoal(30);
            motor_front.motorDirectionControl(0);
            motor_back.motorDirectionControl(0);
            motor_left.motorDirectionControl(1);
            motor_right.motorDirectionControl(0);
            shoot = 0;//wait(0.5);
        }
        
        
        if(mode_value == RIGHT_CLOCK)
        {
            motor_horizontal.motorSpeedControl(0);  
            motor_vertical.motorSpeedControl(0);
            pid_front.setSpeedGoal(30);
            pid_back.setSpeedGoal(30);
            pid_left.setSpeedGoal(30);
            pid_right.setSpeedGoal(30);
            
            motor_front.motorDirectionControl(1);
            motor_back.motorDirectionControl(1);
            motor_left.motorDirectionControl(1);
            motor_right.motorDirectionControl(1);            
            shoot = 0;
        }

        //Cannon series motions
        if(mode_value == CANNON_LEFT)
        {
            motor_horizontal.motorSpeedControl(100);  
            motor_horizontal.motorDirectionControl(0);
            motor_vertical.motorSpeedControl(0);
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(0);
            pid_right.setSpeedGoal(0);  
            shoot = 0;
        }
        if(mode_value == CANNON_RIGHT)
        {
            motor_horizontal.motorSpeedControl(100);  
            motor_horizontal.motorDirectionControl(1);
            motor_vertical.motorSpeedControl(0);  
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(0);
            pid_right.setSpeedGoal(0);
            shoot = 0;
        }
        if(mode_value == CANNON_DOWN)
        {
            motor_vertical.motorSpeedControl(100);  
            motor_vertical.motorDirectionControl(0);
            motor_horizontal.motorSpeedControl(0); 
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(0);
            pid_right.setSpeedGoal(0);
            shoot = 0;
        }
        if(mode_value == CANNON_UP)
        {
            motor_vertical.motorSpeedControl(100);  
            motor_vertical.motorDirectionControl(1);
            motor_horizontal.motorSpeedControl(0); 
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(0);
            pid_right.setSpeedGoal(0);
            shoot = 0;
        }
        
        if(mode_value == SHOOT)
        {
            motor_vertical.motorSpeedControl(0);  
            motor_vertical.motorDirectionControl(1);
            motor_horizontal.motorSpeedControl(0); 
            pid_front.setSpeedGoal(0);
            pid_back.setSpeedGoal(0);
            pid_left.setSpeedGoal(0);
            pid_right.setSpeedGoal(0);
            shoot = 1;
        }
        /*
        pid_front.setSpeedGoal(40);
        pid_back.setSpeedGoal(40);
        pid_left.setSpeedGoal(40);
        pid_right.setSpeedGoal(40);
        */
        if ( pid_front.updateSpeed( &debugger, &front_cmd_speed )) {
            motor_front.motorSpeedControl(front_cmd_speed);      
        }   
                
        if ( pid_back.updateSpeed( &debugger, &back_cmd_speed )) {
            //pc.printf("back:%d\n\r", back_cmd_speed);
            motor_back.motorSpeedControl(back_cmd_speed);      
        }  
        
        if ( pid_left.updateSpeed( &debugger, &left_cmd_speed )) {
            //led4 = 1;
            motor_left.motorSpeedControl(left_cmd_speed);      
        } 
        
        if ( pid_right.updateSpeed( &debugger, &right_cmd_speed )) {
            //led2 = 1;
            //pc.printf("right:%d\n\r", right_cmd_speed);
            motor_right.motorSpeedControl(right_cmd_speed);      
        }    
        }       
        
        if (estop.read() > 1){  
            motor_front.motorSpeedControl(0); 
            motor_back.motorSpeedControl(0);  
            motor_left.motorSpeedControl(0); 
            motor_right.motorSpeedControl(0); 
            motor_vertical.motorSpeedControl(0);  
            motor_horizontal.motorSpeedControl(0);
            shoot = 0;
            flag_recv = 0;
            led4 = 1; 
        }
        
    }
}



