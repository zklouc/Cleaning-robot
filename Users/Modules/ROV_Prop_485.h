/**
  ******************************************************************************
  * @file    ROV_Prop_485.h
  * @author  OUC Fab U+/ROV Team
  * @brief   无刷无感驱动器RS485通信控制模块头文件
  *          基于《无刷无感驱动器操作手册(9.8)》定义通信接口和寄存器地址
  * @version V1.1.0
  * @date    2026-07-11
  ******************************************************************************
  * @attention
  *
  * 根据《无刷无感驱动器操作手册(9.8)》第5章通信协议规范定义：
  * 1. 寄存器地址参考手册第5.2节寄存器映射表
  * 2. 命令值参考手册第5.2.5节命令定义
  * 3. RS485方向控制使用GPIO引脚
  *
  ******************************************************************************
  */

#ifndef ROV_PROP_485_H
#define ROV_PROP_485_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "Global_Define.h"


/**
 * @brief RS485使用的UART句柄
 * @note  使用UART5进行RS485通信
 */
#define RS485_UART       huart5

/**
 * @brief 驱动器数量和缓冲区长度配置
 * @note  MAX_MOTOR_ID: 最多支持的驱动器数量（1~4）
 *        MOD_TX_BUF_LEN: Modbus发送缓冲区最大长度
 */
#define MAX_MOTOR_ID      4
#define MOD_TX_BUF_LEN    64

/**
 * @brief 驱动器寄存器地址定义（参考手册第5.2节寄存器映射表）
 * @note  所有寄存器均为16位，数据格式为大端序
 */
#define REG_CMD_CTRL      0x0131   /* 运行控制命令寄存器（写） */
#define REG_SPEED_SET     0x00AF   /* 速度设定寄存器（写） */
#define REG_SPEED_ACTUAL  0x00B1   /* 实际转速寄存器（读） */
#define REG_CURRENT_ACTUAL 0x00B2  /* 实际电流寄存器（读） */
#define REG_STATUS        0x00B3   /* 运行状态寄存器（读） */
#define REG_ERROR_CODE    0x00B4   /* 错误码寄存器（读） */

/**
 * @brief 控制命令值定义（参考手册第5.2.5节命令定义）
 */
#define CMD_START        0x000F    /* 启动命令：使电机开始运行 */
#define CMD_STOP_SLOW    0x000E    /* 减速停止命令：按设定减速时间停止 */

/**
 * @brief 驱动器状态结构体
 * @details 根据《无刷无感驱动器操作手册(9.8)》第5.2节寄存器映射表定义
 *          用于存储驱动器运行状态的关键参数
 */
typedef struct
{
    int16_t actual_rpm;     /* 实际转速，单位RPM，范围[-3000, 3000] */
    int16_t actual_current; /* 实际电流，单位0.1A，范围[-300, 300]即-30A~30A */
    uint8_t status;         /* 运行状态，参考手册第5.2.3节状态位定义 */
    uint16_t error_code;    /* 错误码，参考手册第5.2.4节错误码定义 */
} Driver_Status_t;

/**
 * @brief 设置电机转速
 * @details 根据《无刷无感驱动器操作手册(9.8)》第6章电机控制流程实现
 *          两阶段发送：先写转速，再发启动/停止控制命令
 * 
 * @param dev_id 设备索引（0~3）
 * @param rpm 目标转速（RPM），范围[-1000, 1000]
 */
void Motor_Set(uint8_t dev_id, int16_t rpm);

/**
 * @brief 读取驱动器状态
 * @details 根据《无刷无感驱动器操作手册(9.8)》第5.2节寄存器映射表实现
 *          连续读取4个寄存器：实际转速、实际电流、状态、错误码
 * 
 * @param dev_id 设备索引（0~3）
 * @param status 输出参数，驱动器状态结构体指针
 * @return 0: 成功，1: 参数错误，2: 通信失败，3: 数据长度不足
 */
uint8_t Motor_GetStatus(uint8_t dev_id, Driver_Status_t *status);

/**
 * @brief 获取上次读取的驱动器状态缓存
 * @note  不进行实际通信，直接返回上次Motor_GetStatus读取的缓存数据
 * 
 * @param dev_id 设备索引（0~3）
 * @return 驱动器状态结构体指针，参数无效时返回NULL
 */
Driver_Status_t *Motor_GetLastStatus(uint8_t dev_id);

/**
 * @brief 模块初始化函数
 * @note  预留接口，当前无需特殊初始化
 */
void Mod_Init(void);

#endif
