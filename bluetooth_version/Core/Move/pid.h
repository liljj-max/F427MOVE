#ifndef PID_H
#define PID_H

//有待调教
#define PID_KP            100.0f
#define PID_KI            0.1f
#define PID_KD            0.0f
#define PID_OUT_MAX       1000.0f
#define PID_INT_LIMIT     500.0f
#define PID_INTEGRAL_SEPARATE_THR    30.0f   // 新增：积分分离阈值



typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output_max;
    float integral_limit;
	  float integral_separate_thr;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max, float int_limit,float separate_thr);
float PID_Calculate(PID_t *pid, float target, float feedback);
void PID_Reset(PID_t *pid);

#endif
