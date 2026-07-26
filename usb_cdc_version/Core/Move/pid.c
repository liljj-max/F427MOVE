#include "pid.h"
#include "math.h"

//只需要速度环，电流环交给电调，位置环交给ROS

//浅浅封装一个赋值
void  PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max, float int_limit, float separate_thr){
	  pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_max = out_max;
    pid->integral_limit = int_limit;
	  pid->integral_separate_thr = separate_thr;
    pid->integral = 0;
    pid->prev_error = 0;
}

float PID_Calculate(PID_t *pid, float target, float feedback)
{
    float error = target - feedback;
    
      // 积分分离：误差太大时不积分，防止积分饱和
    if (fabs(error) < pid->integral_separate_thr) {
        pid->integral += error;
        // 积分限幅
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
    }
    
	//Kd项算斜率
    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;
    
    if(output > pid->output_max) output = pid->output_max;
    if(output < -pid->output_max) output = -pid->output_max;
    
    return output;
}


void PID_Reset(PID_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
}
