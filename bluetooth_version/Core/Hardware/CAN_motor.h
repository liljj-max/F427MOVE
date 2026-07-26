#ifndef MOTOR_CAN2_H
#define MOTOR_CAN2_H

#include "can_protocol.h"

extern MotorFeedback_t motor_fb[MOTOR_NUM];
extern volatile uint8_t can2_error_flag;

void MotorCAN2_SendCurrent(int16_t current1, int16_t current2, 
                              int16_t current3, int16_t current4);
void MotorCAN2_ProcessRx(uint32_t rx_id, uint8_t *data);
void MotorCAN2_ResetBus(void);

#define MOTOR_FEED_LOST_LIMIT  50     // 连续 50 次(500ms)没收到反馈，判离线
void Motor_CheckOnline(void);         // 每次控制周期开始前调用
uint8_t Motor_IsOnline(uint8_t motor_id);  // 查询指定电机是否在线

#endif
