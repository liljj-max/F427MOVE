#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

// ============ USB2字节帧头（ STM32到工控机）============
#define FRAME_HEADER  0xAA55         // 帧头 0xAA55，用来做同步
#define CMD_MAGIC     0xBB           //回传用reserved做校验
// ============ CAN2 ID定义（STM32 ←→ M3508电机）============
#define CAN2_M3508_CMD_BASE  0x200   // 指令基址：0x200 + 电机ID(1~4)
#define CAN2_M3508_FB_BASE   0x200   // 反馈基址：0x200 + 电机ID，实际ID=0x201~0x204

// ============ 电机数量 ============
#define MOTOR_NUM 4

// ============ 错误码定义 ============
#define ERROR_NONE              0x00
#define ERROR_CAN1_BUSOFF       0x01   //总线关闭，发送错误计数器 > 255，CAN 控制器自动离线
#define ERROR_CAN1_EPASSIVE     0x02   //被动错误，发送错误计数器 > 127，进入被动错误状态
#define ERROR_CAN1_EWARNING     0x04   //警告，发送或接收错误计数器 > 96，进入警告状态
#define ERROR_CAN2_BUSOFF       0x10
#define ERROR_CAN2_EPASSIVE     0x20
#define ERROR_CAN2_EWARNING     0x40

#pragma pack(push,1) //一字节对齐

//工控机——STM（ID=0x100)
typedef struct {
    int16_t vx;         // 前进速度 (mm/s)
    int16_t vy;         // 横向速度 (mm/s)
    int16_t wz;         // 角速度 (mrad/s)
    uint8_t enable;     // 使能：1=运行，0=急停
    uint8_t reserved;
} ChassisCmd_t;


//STM——工控（0x101）
typedef struct {
	 uint16_t header;         // 帧头 0xAA55，用来做同步
    uint32_t timestamp_ms;   // 时间戳
    int16_t vx_actual;       // 实际前进速度 (mm/s)
    int16_t vy_actual;       // 实际横向速度 (mm/s)
    uint8_t error_code;      // 故障码
    uint8_t can2_status;     // CAN2状态
} ChassisFeedback_t;

//STM——电调（0x201~0x204)
typedef struct {
    uint16_t angle;          // 转子机械角度 (0-8191)
    int16_t speed_rpm;       // 实际转速 (RPM)
    int16_t current;         // 实际电流 (mA)
    uint8_t temp;            // 温度 (℃)
} MotorFeedback_t;

#pragma pack(pop)

#endif















