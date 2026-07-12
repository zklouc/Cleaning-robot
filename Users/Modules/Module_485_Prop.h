#ifndef MODULE_485_PROP_H
#define MODULE_485_PROP_H

#include <stdint.h>
#include <string.h>
#include "Global_Define.h"

#define RS485_UART       huart5
#define RS485_PROP_MAX_ID     4
#define MOTOR_SPEED_MAX  1000  // 转速最大限幅

// ===================== 寄存器地址（手册标准地址） =====================
// 可写寄存器
#define REG_SPEED_SET    0x00AFU   // 数字调速目标转速 P175（int16）
#define REG_CMD_CTRL     0x0131U   // 虚拟控制命令寄存器

// 只读监视寄存器
#define REG_SPEED_ACTUAL   0x4000U   // 实际转速（int16，1个寄存器）
#define REG_CURRENT_ACTUAL 0x4004U   // 实际电流（float，2个寄存器）
#define REG_ERR_CODE       0x011EU   // 当前报警码（uint16，1个寄存器）

// 控制命令值
#define CMD_START        0x000FU   // 启动电机
#define CMD_STOP_SLOW    0x000EU   // 减速停止
#define CMD_STOP_FREE    0x002EU   // 自由停止

// 通信错误码（直接透传，无额外解码）
typedef enum
{
    MOD_OK = 0,          // 通信正常
    MOD_ERR_TX,          // 串口发送失败
    MOD_ERR_TIMEOUT,     // 接收超时/帧长度不足
    MOD_ERR_CRC,         // CRC校验失败
    MOD_ERR_MATCH,       // 应答地址/功能码/参数不匹配
    MOD_ERR_EXCEPTION,   // 驱动器返回Modbus异常（异常码可单独读取）
    MOD_ERR_PARAM        // 入参非法
} ModErr_t;

// 电机运行状态
typedef enum
{
    MOTOR_STOPPED = 0,   // 停止
    MOTOR_RUNNING = 1,   // 运行中
    MOTOR_ERROR   = 2,   // 故障
} MotorState_t;

// 驱动器状态结构体
typedef struct
{
    int16_t  target_rpm;     // 目标转速（最后一次设定的值）
    int16_t  actual_rpm;     // 实际转速
    float    actual_current; // 实际电流（单位A）
    uint16_t error_code;     // 驱动器原始报警码
    MotorState_t state;      // 电机运行状态
    uint8_t  last_exception; // 上次Modbus异常码
} RS485_Prop_CtrlTypeDef;

// ===================== 核心接口 =====================
// 底层读写寄存器（带完整应答校验）
ModErr_t WriteReg(uint8_t dev_id, uint16_t reg, int16_t value);
ModErr_t ReadReg(uint8_t dev_id, uint16_t reg, uint16_t cnt, uint8_t *data_out);

// 电机控制接口（直接返回通信结果，启动失败可直接判断）
ModErr_t Motor_SetSpeed(uint8_t dev_id, int16_t rpm); // 设置转速
ModErr_t Motor_Start(uint8_t dev_id);                 // 启动推进器
ModErr_t Motor_StopSlow(uint8_t dev_id);              // 减速停止

// 状态与异常读取
ModErr_t Motor_GetStatus(uint8_t dev_id);
RS485_Prop_CtrlTypeDef *Motor_GetLastStatus(uint8_t dev_id); // 获取缓存状态（无通信）
uint8_t  Motor_GetLastException(uint8_t dev_id);      // 获取上次Modbus异常码

extern uint8_t Horizontal_Prop_ID[4];

#endif
