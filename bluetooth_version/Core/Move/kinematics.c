#include "kinematics.h"
#include <math.h>

//       车头 (X+)
//    1 -------- 2
//    |         |
//    |    ○    |
//    |         |
//    4 -------- 3

// 逆解：目标底盘速度 → 各轮 RPM
void Kinematics_Calculate(ChassisSpeed_t cmd, int16_t *wheel_rpm_out)
{
    float vx = cmd.vx / 1000.0f;
    float vy = cmd.vy / 1000.0f;
    float wz = cmd.wz / 1000.0f;

    float d = sqrtf(HALF_LENGTH * HALF_LENGTH + HALF_WIDTH * HALF_WIDTH);
    float vx_proj = vx * SQRT2_INV;          // vx 投影到 45° 轮子方向
    float vy_proj = vy * SQRT2_INV;          // vy 投影到 45° 轮子方向
    float rot_coeff = (HALF_LENGTH + HALF_WIDTH) * SQRT2_INV;
    float rot_term = rot_coeff * wz;

    float v1 =  vx_proj - vy_proj - rot_term;   // 左前 (driving NE)
    float v2 =  vx_proj + vy_proj + rot_term;   // 右前 (driving SE)
    float v3 = -vx_proj + vy_proj - rot_term;   // 右后 (driving SW)
    float v4 = -vx_proj - vy_proj + rot_term;   // 左后 (driving NW)

    wheel_rpm_out[0] = (int16_t)(v1 * RPM_FACTOR);
    wheel_rpm_out[1] = (int16_t)(v2 * RPM_FACTOR);
    wheel_rpm_out[2] = (int16_t)(v3 * RPM_FACTOR);
    wheel_rpm_out[3] = (int16_t)(v4 * RPM_FACTOR);
}

// 正解：实际轮子 RPM → 底盘速度
void Kinematics_Forward(int16_t *wheel_rpm_in, ChassisSpeed_t *speed_out)
{
    float v1 = wheel_rpm_in[0] / RPM_FACTOR;
    float v2 = wheel_rpm_in[1] / RPM_FACTOR;
    float v3 = wheel_rpm_in[2] / RPM_FACTOR;
    float v4 = wheel_rpm_in[3] / RPM_FACTOR;

    float d = sqrtf(HALF_LENGTH * HALF_LENGTH + HALF_WIDTH * HALF_WIDTH);

    // 从正确逆解反推正解
    float vx = ( v1 + v2 - v3 - v4) / (4.0f * SQRT2_INV);
    float vy = (-v1 + v2 + v3 - v4) / (4.0f * SQRT2_INV);
    float wz = ( v2 + v4 - v1 - v3) / (4.0f * d);

    speed_out->vx = (int16_t)(vx * 1000.0f);
    speed_out->vy = (int16_t)(vy * 1000.0f);
    speed_out->wz = (int16_t)(wz * 1000.0f);
}
