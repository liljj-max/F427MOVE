#include "CAN_motor.h"
#include "stm32f4xx_hal.h"

// ===== 新增：丢帧计数器 =====
static uint16_t motor_feed_loss[MOTOR_NUM] = {0};

extern CAN_HandleTypeDef hcan2;

MotorFeedback_t motor_fb[MOTOR_NUM]={0};
volatile uint8_t can2_error_flag=0;


void MotorCAN2_SendCurrent(int16_t current1, int16_t current2,
                              int16_t current3, int16_t current4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    // DJI C620 电调只认 0x200
    tx_header.StdId = 0x200;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    // 4 路电流打包，大端
    tx_data[0] = (current1 >> 8) & 0xFF;
    tx_data[1] = current1 & 0xFF;
    tx_data[2] = (current2 >> 8) & 0xFF;
    tx_data[3] = current2 & 0xFF;
    tx_data[4] = (current3 >> 8) & 0xFF;
    tx_data[5] = current3 & 0xFF;
    tx_data[6] = (current4 >> 8) & 0xFF;
    tx_data[7] = current4 & 0xFF;

    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox);
}

//接收电机回传的消息
void MotorCAN2_ProcessRx(uint32_t rx_id, uint8_t *data)//大端小端存储模式的问题
{
    uint8_t motor_idx = rx_id - CAN2_M3508_FB_BASE;
    if(motor_idx < 1 || motor_idx > MOTOR_NUM) return;
    
    motor_fb[motor_idx-1].angle     = (uint16_t)((data[0] << 8) | data[1]);
    motor_fb[motor_idx-1].speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    motor_fb[motor_idx-1].current   = (int16_t)((data[4] << 8) | data[5]);
    motor_fb[motor_idx-1].temp      = data[6];
	
	motor_feed_loss[motor_idx-1] = 0;    // 收到反馈就清零
}

//犯唐被关小黑屋以后，又回归主动错误状态
void MotorCAN2_ResetBus(void)
{
    HAL_CAN_ResetError(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

// ===== 新增：每次控制周期调用，更新丢帧计数 =====
void Motor_CheckOnline(void)
{
    for(int i = 0; i < MOTOR_NUM; i++)
    {
        if(motor_feed_loss[i] < 0xFFFF)
            motor_feed_loss[i]++;
    }
}


// ===== 新增：查询电机是否在线 =====
uint8_t Motor_IsOnline(uint8_t motor_id)
{
    if(motor_id < 1 || motor_id > MOTOR_NUM) return 0;
    return (motor_feed_loss[motor_id - 1] < MOTOR_FEED_LOST_LIMIT) ? 1 : 0;
}






















