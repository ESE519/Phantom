#include "mbed.h"
#include "msg_interface.h"
#include "message_definitions.h"
#include "queue.h"
#include "IMU_MPU6050.h"
#include "MPU6050.h"


#define position_num 4//yellow
#define sample_num 10
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
Timer t;

MPU6050 imu_mpu6050(p28, p27);      // p28 sda, p27 scl, define the IMU connection 
//float s[position_num][sample_num]={{0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25},{0.35,0.48,0.65,0.84,1,1,1,1,1,0.93,0.89,0.77,0.70,0.60,0.54,0.49,0.44,0.40,0.37,0.35}};
//float s[position_num][sample_num]={{0,0,0,0,0},{1,1,1,1,1}};
//float s[position_num][sample_num]={{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0.35,0.48,0.65,0.84,1,1,1,1,1,0.93,0.89,0.77,0.70,0.60,0.54,0.49,0.44,0.40,0.37,0.35}};246,371,465,813,1000,1000,1000,871,596,547
//float s[position_num][sample_num]={{200,200,200,200,200,200,200,200,200,200},{321,327,427,775,1000,1000,833,598,473,396},{900,900,900,900,900,900,900,900,900,900},{500,500,500,500,500,500,500,500,500,500}};//yellow//321,327,427,775,1000,1000,833,598,473,396
//float s1[position_num][sample_num]={{65,65,65,65,65,65,65,65,65,65},{321,327,427,775,1000,1000,833,598,473,396},{900,900,900,900,900,900,900,900,900,900},{320,320,320,320,320,320,320,320,320,320}};
float s[position_num][sample_num]={{400,400,400,400,400,400,400,400,400,400},{321,327,427,775,1000,1000,833,598,473,396},{900,900,900,900,900,900,900,900,900,900},{800,800,800,800,800,800,800,800,800,800}};
float s1[position_num][sample_num]={{265,265,265,265,265,265,265,265,265,265},{321,327,427,775,1000,1000,833,598,473,396},{900,900,900,900,900,900,900,900,900,900},{800,800,800,800,800,800,800,800,800,800}};
//float s[position_num][sample_num]={{200,200,200,200,200,200,200,200,200,200},{401,631,1000,1000,1000,1000,1000,847,539,366},{900,900,900,900,900,900,900,900,900,900}};//green
AnalogIn ain(p19); 
AnalogIn ain_temp=(p18);

DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);


void sendPackage(void);
void adc_acquire(void);
void adc_acquire_temp(void);
void mode_change(void);

DigitalOut got_packet(LED3);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led4(LED4);
DigitalOut rst(p20);

Ticker send;
Ticker adc_aqr;
Ticker adc_aqr_temp;
Ticker mode_cg;
Ticker get_position;
Ticker get_position_temp;
Ticker DataUpdateIMU;       // define the ticker to update the data from IMU


Serial xbee(p9,p10);
Queue myQueue(4,sample_num);
Queue myQueue_temp(4,sample_num);

float adc;
float adc_temp;
int pos;
int pos_temp;
float updateTimeIMU = 0.01;         // unit is second, update date time
float beta_a = 0.95;        // coefficient for low-pass filter
float beta_g = 0.95;        // coefficient for high-pass filter
float current_position;

bool mode_enable=1;
int mode_value=1;




IMU_MPU6050 imu_hand(&imu_mpu6050, &DataUpdateIMU, updateTimeIMU, beta_a, beta_g);      // define the IMU device

MsgModeSend msg;
MessageIface msgIface(&xbee);

void init()
{
    int sum=0;
    int sum_temp=0;
     for(int i=0;i<1000;i++)
     {
        sum+=adc;
        sum_temp+=adc_temp;
     }
     int ave=sum/1000;
     int ave_temp=sum_temp/1000;
     for(int i=0;i<10;i++)
     {
        s[0][i]=ave;
        s1[0][i]=ave_temp;
     }
}


float LSR(float s1[],float s2[])
{
    float err=0;
    for(int i=0;i<sample_num;i++)
    {
       int diff=(int)(s1[i]-s2[i]);
       err +=(float)(diff*diff);
    }
    return err;
}

void position_decide()
{
   float a[sample_num];
   float error[sample_num];
   for(int i=0;i<sample_num;++i)
   {
      myQueue.Get(&a[i]);
   }
   int min=0;
   for(int j=0;j<position_num;++j)
   {
     error[j]=LSR(a,s[j]);
   }
   for(int j=0;j<position_num;++j)
   {
     if(error[j]<error[min])
     {
       min=j;
     }
   }
   pos=min;
}

void position_decide_temp()
{
   float a[sample_num];
   float error[sample_num];
   for(int i=0;i<sample_num;++i)
   {
      myQueue_temp.Get(&a[i]);
   }
   int min=0;
   for(int j=0;j<position_num;++j)
   {
     error[j]=LSR(a,s1[j]);
   }
   for(int j=0;j<position_num;++j)
   {
     if(error[j]<error[min])
     {
       min=j;
     }
   }
   
   pos_temp=min;
}



int main() {
   
    pc.baud(115200);

    float temp;

    pc.printf("Hello!");
    
    //implement XBee communication
    send.attach_us(&sendPackage,10000);
    //implement EMG signal processing
    adc_aqr.attach(&adc_acquire,0.05);
    adc_aqr_temp.attach(&adc_acquire_temp,0.05);
    get_position.attach(&position_decide,0.05);
    get_position_temp.attach(&position_decide_temp,0.05);

    xbee.baud(57600);
    rst = 0;
    led1 = 0;
    wait_ms(1);
    rst = 1;
    wait_ms(1);
    led1 = 1;
    
    led2 = 0;
    // xbee send data 
    
    msg.mode_send = 1;
    msgIface.sendPacket( MsgTypeModeSend, (uint8_t*)&msg, sizeof(MsgModeSend) );        
    while ( !msgIface.sendComplete() ) {
        msgIface.sendNow();
    }
    wait_ms(250);
    led2 = 1;
    
    wait_ms(250);
    led2 = 0;
    
    msg.mode_send = 1;
    msgIface.sendPacket( MsgTypeModeSend, (uint8_t*)&msg, sizeof(MsgModeSend) );        
    while ( !msgIface.sendComplete() ) {
        msgIface.sendNow();
    }
    wait_ms(250);
    led2 = 1;
    
    
    wait_ms(250);
    led2 = 0;
    
    msg.mode_send = 1;
    msgIface.sendPacket( MsgTypeModeSend, (uint8_t*)&msg, sizeof(MsgModeSend) ); 
    //Implement IMU function
    imu_mpu6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);    // set the scale of the accelerometer
    imu_mpu6050.setGyroRange(MPU6050_GYRO_RANGE_250);           // set the scale of the gyro
    float angle_x;
    float angle_y;      // store the angle of x-axis and angle of y-axis
    bool success_flag;  // indicate the iic connection
    
    success_flag = imu_hand.testCommunication();    // if successful, return 1
    //train and get the train sample
    init();
    while(!success_flag)        // wait until connection
    {
        myled = 1;
    }  
 // while loop  

    t.start();
    while(1) {
        imu_hand.getAngleXY(&angle_x, &angle_y);    // get angle_x and angle_y      
        myQueue.Put(&adc);
        myQueue_temp.Put(&adc_temp);
        //pc.printf("y:  %f",angle_y);
        if(pos==2&&pos==2&& angle_y>65)
        {
            
            if(mode_enable==1)
            {
              t.reset();
              mode_enable=0;
              pc.printf("SHOOT\r\n");
              msg.mode_send = SHOOT;
              
            }
              
        }
       // pc.printf("time : %f\r\n", t.read());
       // pc.printf("mode_enable : %d\r\n", mode_enable);
        if(t.read()>3)
        {
           t.reset();
           mode_enable=1;
        }
        
        if(pos==0&&pos_temp==0)
        {
           pc.printf("IDLE\r\n");
           msg.mode_send = 0;         
        }

        else
        {
          
               if(adc-adc_temp>80)
               {
                  if(abs(angle_x)<30)
                  {
                    pc.printf("CANNON_UP\r\n");
                      msg.mode_send = CANNON_UP;
                  }
                  if(angle_x>60 && angle_x<90)
                  {
                     pc.printf("CANNON_LEFT\r\n");
                      msg.mode_send = CANNON_LEFT;
                  }
                  if(angle_x<-60 && angle_x>-90)
                  {
                     pc.printf("SHOOT\r\n");
                      msg.mode_send = SHOOT;
                  }

               }
               else if(adc_temp-adc>-60)
               {
               
                  if(abs(angle_x)<30)
                  {
                    pc.printf("CANNON_DOWN\r\n");
                      msg.mode_send = CANNON_DOWN;
                  }
                  if(angle_x>60 && angle_x<90)
                  {
                     pc.printf("CANNON_RIGHT\r\n");
                      msg.mode_send = CANNON_RIGHT;
                  }

               }
            
            
        }
       // pc.printf("pos: %d\r\n",pos_temp);
        if(myQueue.GetNumberOfItems( )==sample_num)
        {
           myQueue.Get(&temp);
        }
        if(myQueue_temp.GetNumberOfItems( )==sample_num)
        {
           myQueue_temp.Get(&temp);
        }
        msgIface.sendNow();       
    }
    
}

void sendPackage() {
    msgIface.sendPacket( MsgTypeModeSend, (uint8_t*)&msg, sizeof(MsgModeSend) );
   // pc.printf("%d\r\n",msg.mode_send );
}

void adc_acquire(){
        adc = ain.read();
        adc=adc*1000;
        //pc.printf("adc: %f\r\n",adc);
 }
 
 void adc_acquire_temp(){
        adc_temp = ain_temp.read();
        adc_temp=adc_temp*1000;
        //pc.printf("adc_temp: %f\r\n",adc_temp);
 }
