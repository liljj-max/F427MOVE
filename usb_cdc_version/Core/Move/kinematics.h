
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#define PI 3.1415926535f
#define WHEEL_RADIUS  0.049   //全向轮半径(m)
#define HALF_LENGTH   0.244    // X方向半长 
#define HALF_WIDTH    0.220     // Y方向半宽 
#define SQRT2_INV 0.70710678f
#define RPM_FACTOR  60.0f/(2.0f*PI*WHEEL_RADIUS)
#define CMD_TIMEOUT_MS    200       // 200ms无指令停车

typedef struct {
    int16_t vx;     // 前进速度 (mm/s)
    int16_t vy;     // 横向速度 (mm/s，左正右负)
    int16_t wz;     // 角速度 (mrad/s，逆时针正)
} ChassisSpeed_t;

//void Kinematics_Init(float wheel_radius_m, float robot_radius_m);
void Kinematics_Calculate(ChassisSpeed_t cmd, int16_t *wheel_rpm_out);
// 新增：从实际轮子转速反算底盘速度
void Kinematics_Forward(int16_t *wheel_rpm_in, ChassisSpeed_t *speed_out);

#endif
