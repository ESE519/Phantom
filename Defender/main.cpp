#include "mbed.h"
#include "msg_interface.h"
#include "message_definitions.h"


#define IDLE 0
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

MessageIface msgIface(&xbee);
MsgMode *msg_mode;      // define the structure for receiving

PwmOut motor_right_enable(p21);
PwmOut motor_left_enable(p22);
PwmOut motor_down_enable(p23);
PwmOut motor_up_enable(p24);
// shoot
DigitalOut shoot(p25);

Timer estop;
bool flag_recv;

int main() {

    
    // xbee received data 
    uint8_t *rx_data;
    uint8_t rx_len;
    uint8_t msg_type;    
    uint8_t *msg_data;
    uint8_t mode_value;
    
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
    
    motor_right_enable.period_ms(20);
    motor_left_enable.period_ms(20);
    motor_up_enable.period_ms(20);
    motor_down_enable.period_ms(20);
    shoot = 0;
    //wait(1);
    //motor_left_enable.pulsewidth_ms(20);
    //motor_right_enable.pulsewidth_ms(10);
    //motor_up_enable.pulsewidth_ms(20);
    //motor_down_enable.pulsewidth_ms(20);
    //shoot = 1;
    //wait(2);
    //motor_left_enable.pulsewidth_ms(0);
    //motor_right_enable.pulsewidth_ms(0);
    //motor_up_enable.pulsewidth_ms(0);
    //motor_down_enable.pulsewidth_ms(0);
    //shoot = 0;
    
    

    while(1) {
        //led4 = 1;
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
                break;
               
            }
            
            msgIface.dropPacket();  
        }
        pc.printf("mode: %d\n\r", mode_value);
        

        
        if(flag_recv) {

        //Cannon series motions
        if (mode_value == IDLE)
        {
            motor_left_enable.pulsewidth_ms(0);
            motor_right_enable.pulsewidth_ms(0);
            motor_up_enable.pulsewidth_ms(0);
            motor_down_enable.pulsewidth_ms(0);
            shoot = 0;
            led1 = 0;
        }
        if(mode_value == CANNON_LEFT)
        {
            motor_left_enable.pulsewidth_ms(20);
            motor_right_enable.pulsewidth_ms(0);
            motor_up_enable.pulsewidth_ms(0);
            motor_down_enable.pulsewidth_ms(0);
            shoot = 0;
            led1 = 0;
        }
        if(mode_value == CANNON_RIGHT)
        {
            motor_left_enable.pulsewidth_ms(0);
            motor_right_enable.pulsewidth_ms(20);
            motor_up_enable.pulsewidth_ms(0);
            motor_down_enable.pulsewidth_ms(0);
            shoot = 0;
            led1 = 0;
        }
        if(mode_value == CANNON_DOWN)
        {
            motor_left_enable.pulsewidth_ms(0);
            motor_right_enable.pulsewidth_ms(0);
            motor_up_enable.pulsewidth_ms(0);
            motor_down_enable.pulsewidth_ms(20);
            shoot = 0;
            led1 = 1;
        }
        if(mode_value == CANNON_UP)
        {
            motor_left_enable.pulsewidth_ms(0);
            motor_right_enable.pulsewidth_ms(0);
            motor_up_enable.pulsewidth_ms(20);
            motor_down_enable.pulsewidth_ms(0);
            shoot = 0;
            led1 = 0;
        }
        
        if(mode_value == SHOOT)
        {
            motor_left_enable.pulsewidth_ms(0);
            motor_right_enable.pulsewidth_ms(0);
            motor_up_enable.pulsewidth_ms(0);
            motor_down_enable.pulsewidth_ms(0);
            shoot = 1;
            led1 = 0;
        }  
        }       
        
        if (estop.read() > 1){  
            motor_left_enable.pulsewidth_ms(0);
            motor_right_enable.pulsewidth_ms(0);
            motor_up_enable.pulsewidth_ms(0);
            motor_down_enable.pulsewidth_ms(0);
            shoot = 0;
            flag_recv = 0;
            led4 = 1; 
        }
        
    }
}



