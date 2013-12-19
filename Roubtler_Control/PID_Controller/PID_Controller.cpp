#include "PID_Controller.h"
#include "message_definitions.h"


PIDController::PIDController(uint8_t id, QEI *ENCODER, Ticker *DATAUpdate, float updateTIME, float KP, float KI, float KD)
{
    _id = id;
    Kp = KP;
    Kd = KD;
    Ki = KI;
    previousSpeed = 0;
    prev_derivative = 0;
    alpha = 0.1;
    previous_error = 0;
    setPosition = 0;
    setSpeed = 0;
    integral = 0;
    currentPosition_previous = 0;
    currentSpeed = 0;
    encoder = ENCODER;
    DataUpdate = DATAUpdate;
    updateTime = updateTIME;
    //QEI encoder
    DataUpdate->attach(this,&PIDController::UpdateData,updateTime);
    led2 = new DigitalOut(LED2);
    
}

void PIDController::UpdateData(void) 
{
    currentPosition = encoder->getPulses();
        
    flag_update_PID = 1;
    //printf("cPos: %d\n\r", currentPosition);
    //led2->write(1);
    //wait(0.05);
    //led2->write(0);
    //wait(0.05);
}

int PIDController::return_currentPosition()
{
    return currentPosition;
}

float PIDController::return_currentSpeed()
{
    return currentSpeed;
}

int PIDController::updateSpeed( Debugger* debug_iface, signed char *cmdSpeed)
{
    if (flag_update_PID == 0)
    {
        return 0;
    }
    else
    {
        
        float dt = updateTime;   // convert the time from ms into s
        //pc->printf("update: %f\n\r",updateTime);
        //pc->printf("dt: %f\n\r", dt);
        //float integral = 0;
        float derivative = 0;
        signed char output = 0;
        int output_int = 0;
        //printf("cP: %d\n\r", currentPosition);
        currentSpeed = -((float)(currentPosition - currentPosition_previous)) / (64 * dt) * 60;    // unit is RPM
        
        // Speed low-pass 
        currentSpeed = 0.2*currentSpeed + (1-0.2)*previousSpeed;
        previousSpeed = currentSpeed;
        
        //printf("sP: %f\n\r", (float)setPosition);
        //pc->printf("cS: %f\n\r", currentSpeed);
        float error = (float)setSpeed - 100/150*currentSpeed;
        //pc->printf("Err: %f\n\r", error);
        integral = integral + error * dt;
        //pc->printf("PErr: %f\n\n", previous_error);
        // Derivative with low-pass
        derivative = (error - previous_error) / dt;
        //derivative = alpha*derivative + (1-alpha)*prev_derivative;
        //prev_derivative = derivative;
        
        //pc->printf("d: %f\n\r", derivative);
        previous_error = error;
        currentPosition_previous = currentPosition;
        //pc->printf("KP: %f\n\r", Kp);
        //pc->printf("P: %f\n\r", Kp*error);
        //pc->printf("I: %d\n\r", Ki);
        //pc->printf("D: %d\n\r", Kd);
        output_int = setSpeed + Kp * error + Ki * integral + Kd * derivative + 0.5; // plus 0.5 to round the float number
        //printf("OUT: %d\n\r", output_int);
        if(output_int > 127) {
            output_int = 127;
        }
        if(output_int < -127) {
            output_int = -127;
        }
        /*
        if(output_int > -5 && output_int < 5)
        {
            output_int = 0;
        }
        */
        output = output_int;
        //output = 0;
        //pc->printf("O: %d\n\r", output);
        
        // Set feedback message
        
        /*MsgMotorFeedback msg_motor_feedback;
        msg_motor_feedback.id = _id;
        msg_motor_feedback.wheel_cmd = setPosition;
        msg_motor_feedback.wheel_speed = currentSpeed;
        debug_iface->sendPacket( MsgTypeMotorFeedback, (uint8_t*)&msg_motor_feedback, sizeof(MsgMotorFeedback) );
        */
        
        *cmdSpeed = output; 
        flag_update_PID = 0;
        return 1;
     }
}

void PIDController::setSpeedGoal(int8_t goalSpeed)
{   
    setSpeed = goalSpeed; 
}
void PIDController::setPositionGoal(int goalPosition)
{
    setPosition = goalPosition;
}

int PIDController::update(Serial *pc, int *cmd)
{
    if (flag_update_PID == 0)
    {
        return 0;
    }
    else
    {
        float dt = updateTime;
        //float integral = 0;
        float derivative = 0;
        int output = 0;
        //pc->printf("dt: %f\n\r", dt);
        //pc->printf("sP: %d\n\r", setPosition);
        //pc->printf("cP: %d\n\r", currentPosition);
        
        float error = (float)(setPosition - currentPosition) / 3360 * 180 ;
        
        //pc->printf("Err: %f\n\r", error);
        integral = integral + error * dt;
        derivative = ((float)(error - previous_error)) / dt;
        previous_error = error;
        output = int((float)setPosition / 3360 * 180 + Kp * error + Ki * integral + Kd * derivative + 0.5);
        //pc->printf("P: %f\n\r", Kp * error);
        //pc->printf("I: %f\n\r", integral);
        //pc->printf("D: %f\n\r", Kd * derivative);
        //pc->printf("OFFSET: %f\n\r", (float)setPosition / 3360 * 180);
        //pc->printf("out: %d\n\r", output);
        
        if (output < 18 && output > 3)    // deadband
        {
            output = 18;
        }
        if (output > -18 && output < -3)
        {
            output = -18;
        }
        
        
       
        if(output > 127) {
            output = 127;
        }
        if(output < -127) {
            output = -127;
        }
        //pc->printf("OP: %d\n\r", output);
        //output = 0;
        *cmd = output;
        
           
        flag_update_PID = 0;
        return 1;
     }
}

