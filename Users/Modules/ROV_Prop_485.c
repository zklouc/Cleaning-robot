/**
  ******************************************************************************
  * @file    ROV_Prop_485.c
  * @author  OUC Fab U+/ROV Team
  * @brief   无刷无感驱动器RS485通信控制模块
  *          基于《无刷无感驱动器操作手册(9.8)》实现Modbus RTU协议通信
  * @version V1.1.0
  * @date    2026-07-11
  * @note    通信协议：Modbus RTU
  *          物理接口：RS485（UART5）
  *          支持设备：最多4个无刷无感驱动器（地址1~4）
  ******************************************************************************
  * @attention
  *
  * 根据《无刷无感驱动器操作手册(9.8)》第5章通信协议规范实现：
  * 1. 使用功能码0x10（写多个寄存器）发送控制指令
  * 2. 使用功能码0x03（读多个寄存器）读取驱动器状态
  * 3. CRC16校验算法：初始值0xFFFF，多项式0xA001
  * 4. 寄存器地址参考手册第5.2节寄存器映射表
  *
  ******************************************************************************
  */

#include "ROV_Prop_485.h"

/**
 * @brief 通信缓冲区
 * @note  tx_buf: 发送缓冲区，最大16字节（Modbus帧最大11字节）
 *        tx_len: 当前发送帧长度
 *        rx_buf: 接收缓冲区，最大32字节
 *        rx_len: 当前接收数据长度
 */
static uint8_t tx_buf[16];
static uint16_t tx_len;
static uint8_t rx_buf[32];
static uint16_t rx_len;

/**
 * @brief CRC16校验算法
 * @details 根据《无刷无感驱动器操作手册(9.8)》第5.1.3节CRC校验规范实现
 *          标准Modbus CRC16算法：初始值0xFFFF，多项式0xA001（0x8005反转）
 * 
 * @param buf 待计算数据缓冲区
 * @param len 数据长度
 * @return 计算得到的CRC16校验值
 */
static uint16_t CRC16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;  /* CRC初始值，手册规定为0xFFFF */
    
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];  /* 与当前字节异或 */
        
        for (uint8_t j = 0; j < 8; j++) {
            /* 右移1位，最低位为1时与多项式0xA001异或 */
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief 打包Modbus写寄存器请求帧（功能码0x10）
 * @details 根据《无刷无感驱动器操作手册(9.8)》第5.1.2节写寄存器帧格式实现
 *          功能码0x10：写多个寄存器（此处只写1个寄存器）
 * 
 * Modbus RTU写寄存器帧格式（11字节）：
 * | 字节0 | 字节1 | 字节2-3 | 字节4-5 | 字节6 | 字节7-8 | 字节9-10 |
 * |-------|-------|---------|---------|-------|---------|----------|
 * | 从站地址 | 功能码0x10 | 寄存器地址高-低 | 寄存器数量 | 字节数 | 写入值高-低 | CRC低-高 |
 * 
 * @param dev_id 从站地址（1~4）
 * @param reg 寄存器地址（参考手册寄存器映射表）
 * @param value 要写入的值（16位有符号整数）
 */
static void PackWriteReg(uint8_t dev_id, uint16_t reg, int16_t value)
{
    tx_len = 0;
    
    tx_buf[tx_len++] = dev_id;                    /* 字节0: 从站地址 */
    tx_buf[tx_len++] = 0x10;                      /* 字节1: 功能码，写多个寄存器 */
    tx_buf[tx_len++] = (reg >> 8) & 0xFF;         /* 字节2: 寄存器地址高字节 */
    tx_buf[tx_len++] = reg & 0xFF;                /* 字节3: 寄存器地址低字节 */
    tx_buf[tx_len++] = 0x00;                      /* 字节4: 寄存器数量高字节 */
    tx_buf[tx_len++] = 0x01;                      /* 字节5: 寄存器数量低字节（1个） */
    tx_buf[tx_len++] = 0x02;                      /* 字节6: 数据字节数（2字节） */
    tx_buf[tx_len++] = (value >> 8) & 0xFF;       /* 字节7: 写入值高字节 */
    tx_buf[tx_len++] = value & 0xFF;              /* 字节8: 写入值低字节 */
    
    /* 计算CRC并添加到帧尾 */
    uint16_t crc = CRC16(tx_buf, tx_len);
    tx_buf[tx_len++] = crc & 0xFF;                /* 字节9: CRC低字节 */
    tx_buf[tx_len++] = (crc >> 8);                /* 字节10: CRC高字节 */
}

/**
 * @brief 发送Modbus写寄存器指令
 * @details 根据《无刷无感驱动器操作手册(9.8)》第5.3节发送流程实现
 *          1. 打包Modbus帧
 *          2. 发送数据
 *          3. 等待发送完成
 * 
 * @param dev_id 从站地址（1~4）
 * @param reg 寄存器地址
 * @param value 要写入的值
 * @return 0: 发送成功，1: 发送失败（超时或硬件错误）
 */
static uint8_t SendCmd(uint8_t dev_id, uint16_t reg, int16_t value)
{
    /* 打包Modbus写寄存器帧 */
    PackWriteReg(dev_id, reg, value);
    
    /* 通过UART发送数据，超时时间50ms */
    if (HAL_UART_Transmit(&RS485_UART, tx_buf, tx_len, 50) != HAL_OK)
    {
        return 1;  /* 返回错误码1 */
    }
    
    /* 等待发送完成标志（TC: Transmission Complete） */
    while (__HAL_UART_GET_FLAG(&RS485_UART, UART_FLAG_TC) == RESET);
    
    return 0;  /* 发送成功 */
}

/**
 * @brief 打包Modbus读寄存器请求帧（功能码0x03）
 * @details 根据《无刷无感驱动器操作手册(9.8)》第5.1.1节读寄存器帧格式实现
 *          功能码0x03：读多个寄存器
 * 
 * Modbus RTU读寄存器请求帧格式（8字节）：
 * | 字节0 | 字节1 | 字节2-3 | 字节4-5 | 字节6-7 |
 * |-------|-------|---------|---------|----------|
 * | 从站地址 | 功能码0x03 | 起始寄存器地址高-低 | 寄存器数量高-低 | CRC低-高 |
 * 
 * @param dev_id 从站地址（1~4）
 * @param reg 起始寄存器地址
 * @param count 要读取的寄存器数量
 */
static void PackReadReg(uint8_t dev_id, uint16_t reg, uint16_t count)
{
    tx_len = 0;
    
    tx_buf[tx_len++] = dev_id;                    /* 字节0: 从站地址 */
    tx_buf[tx_len++] = 0x03;                      /* 字节1: 功能码，读多个寄存器 */
    tx_buf[tx_len++] = (reg >> 8) & 0xFF;         /* 字节2: 起始寄存器地址高字节 */
    tx_buf[tx_len++] = reg & 0xFF;                /* 字节3: 起始寄存器地址低字节 */
    tx_buf[tx_len++] = (count >> 8) & 0xFF;       /* 字节4: 寄存器数量高字节 */
    tx_buf[tx_len++] = count & 0xFF;              /* 字节5: 寄存器数量低字节 */
    
    /* 计算CRC并添加到帧尾 */
    uint16_t crc = CRC16(tx_buf, tx_len);
    tx_buf[tx_len++] = crc & 0xFF;                /* 字节6: CRC低字节 */
    tx_buf[tx_len++] = (crc >> 8);                /* 字节7: CRC高字节 */
}

/**
 * @brief 发送Modbus读寄存器指令并接收响应
 * @details 根据《无刷无感驱动器操作手册(9.8)》第5.3节收发流程实现
 *          1. 发送读寄存器请求
 *          2. 等待响应（超时100ms）
 *          3. 校验CRC
 * 
 * Modbus RTU读寄存器响应帧格式：
 * | 字节0 | 字节1 | 字节2 | 字节3~N | 字节N+1~N+2 |
 * |-------|-------|-------|---------|-------------|
 * | 从站地址 | 功能码0x03 | 数据字节数 | 寄存器数据 | CRC低-高 |
 * 
 * @param dev_id 从站地址（1~4）
 * @param reg 起始寄存器地址
 * @param count 要读取的寄存器数量
 * @return 0: 成功，1: 发送失败，2: 接收超时/数据不足，3: CRC校验错误
 */
static uint8_t ReadCmd(uint8_t dev_id, uint16_t reg, uint16_t count)
{
    /* 打包Modbus读寄存器帧 */
    PackReadReg(dev_id, reg, count);
    
    /* 发送读请求 */
    if (HAL_UART_Transmit(&RS485_UART, tx_buf, tx_len, 50) != HAL_OK)
    {
        return 1;  /* 发送失败 */
    }
    
    /* 等待发送完成 */
    while (__HAL_UART_GET_FLAG(&RS485_UART, UART_FLAG_TC) == RESET);
    
    /* 清空接收缓冲区 */
    rx_len = 0;
    
    /* 等待响应数据，超时100ms（参考手册通信超时要求） */
    uint32_t timeout = HAL_GetTick();
    while (HAL_GetTick() - timeout < 100)
    {
        /* 检查是否有接收数据 */
        if (__HAL_UART_GET_FLAG(&RS485_UART, UART_FLAG_RXNE) != RESET)
        {
            /* 读取一个字节 */
            if (rx_len < sizeof(rx_buf))
            {
                rx_buf[rx_len++] = (uint8_t)(RS485_UART.Instance->DR & 0xFF);
            }
            /* 收到数据后重置超时计时（支持连续接收） */
            timeout = HAL_GetTick();
        }
    }
    
    /* 检查最小帧长度（地址1 + 功能码1 + 字节数1 + 数据N + CRC2 = 最少5字节） */
    if (rx_len < 5)
        return 2;  /* 接收超时或数据不足 */
    
    /* 校验CRC */
    uint16_t crc_calc = CRC16(rx_buf, rx_len - 2);
    uint16_t crc_recv = (uint16_t)rx_buf[rx_len - 1] << 8 | rx_buf[rx_len - 2];
    if (crc_calc != crc_recv)
        return 3;  /* CRC校验错误 */
    
    return 0;  /* 读取成功 */
}

/**
 * @brief 驱动器状态缓存数组
 * @note  保存每个驱动器的最新状态，供快速查询
 */
static Driver_Status_t driver_status[MAX_MOTOR_ID];

/**
 * @brief 读取驱动器状态
 * @details 根据《无刷无感驱动器操作手册(9.8)》第5.2节寄存器映射表实现
 *          连续读取4个寄存器：实际转速、实际电流、状态、错误码
 * 
 * @param dev_id 设备索引（0~3，内部转换为1~4的从站地址）
 * @param status 输出参数，驱动器状态结构体指针
 * @return 0: 成功，1: 参数错误，2: 通信失败，3: 数据长度不足
 */
uint8_t Motor_GetStatus(uint8_t dev_id, Driver_Status_t *status)
{
    /* 将0-based索引转换为1-based从站地址 */
    uint8_t real_id = dev_id + 1;
    
    /* 参数有效性检查 */
    if (real_id < 1 || real_id > MAX_MOTOR_ID || status == NULL)
        return 1;
    
    /* 读取4个连续寄存器：REG_SPEED_ACTUAL ~ REG_ERROR_CODE */
    uint8_t ret = ReadCmd(real_id, REG_SPEED_ACTUAL, 4);
    if (ret != 0)
        return ret;  /* 返回通信错误码 */
    
    /* 检查响应帧最小长度（地址1 + 功能码1 + 字节数1 + 数据8 + CRC2 = 13字节） */
    if (rx_len < 13)
        return 2;
    
    /* 检查数据字节数是否足够 */
    uint8_t byte_count = rx_buf[2];
    if (byte_count < 8)
        return 3;
    
    /* 解析响应数据（大端序） */
    status->actual_rpm = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[4]);       /* 寄存器0x00B1 */
    status->actual_current = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[6]);   /* 寄存器0x00B2 */
    status->status = rx_buf[7];                                                  /* 寄存器0x00B3 */
    status->error_code = (uint16_t)rx_buf[8] << 8 | rx_buf[9];                   /* 寄存器0x00B4 */
    
    /* 缓存状态数据 */
    memcpy(&driver_status[dev_id], status, sizeof(Driver_Status_t));
    
    return 0;  /* 读取成功 */
}

/**
 * @brief 获取上次读取的驱动器状态缓存
 * @note  不进行实际通信，直接返回上次Motor_GetStatus读取的缓存数据
 * 
 * @param dev_id 设备索引（0~3）
 * @return 驱动器状态结构体指针，参数无效时返回NULL
 */
Driver_Status_t *Motor_GetLastStatus(uint8_t dev_id)
{
    if (dev_id >= MAX_MOTOR_ID)
        return NULL;
    return &driver_status[dev_id];
}

/**
 * @brief 电机状态跟踪变量
 * @note  last_motor_run[i]: 记录第i个电机上一次的运行状态（0=停止，1=运行）
 *        send_ctrl_pending[i]: 控制指令待发送标志（1=待发送启动/停止指令）
 */
static uint8_t last_motor_run[MAX_MOTOR_ID] = {0};
static uint8_t send_ctrl_pending[MAX_MOTOR_ID] = {0};

/**
 * @brief 电机控制参数
 * @note  MOTOR_START_THRESH: 启动阈值，转速绝对值超过此值才允许启动（防止误触发）
 *        MOTOR_SPEED_MAX: 最大转速限制，参考手册最大转速3000RPM，此处设为1000RPM安全值
 */
#define MOTOR_START_THRESH  500   /* 启动阈值：500 RPM */
#define MOTOR_SPEED_MAX     1000  /* 最大转速：1000 RPM */

/**
 * @brief 设置电机转速
 * @details 根据《无刷无感驱动器操作手册(9.8)》第6章电机控制流程实现
 *          控制流程：
 *          1. 参数预处理（地址转换、速度限幅）
 *          2. 运行状态决策（启动/停止逻辑）
 *          3. 两阶段指令发送：
 *             - 阶段1：发送启动/停止控制指令（REG_CMD_CTRL）
 *             - 阶段2：发送转速设定指令（REG_SPEED_SET）
 * 
 * @param dev_id 设备索引（0~3）
 * @param rpm 目标转速（RPM），范围[-1000, 1000]
 */
void Motor_Set(uint8_t dev_id, int16_t rpm)
{
    /* 将0-based索引转换为1-based从站地址 */
    uint8_t real_id = dev_id + 1;
    
    /* 参数有效性检查 */
    if (real_id < 1 || real_id > MAX_MOTOR_ID)
        return;
    
    /* 转速限幅：限制在[-MOTOR_SPEED_MAX, MOTOR_SPEED_MAX]范围内 */
    int16_t set_rpm = rpm;
    if (set_rpm > MOTOR_SPEED_MAX)  set_rpm = MOTOR_SPEED_MAX;
    if (set_rpm < -MOTOR_SPEED_MAX) set_rpm = -MOTOR_SPEED_MAX;
    
    /* 根据当前状态和目标转速计算期望运行状态 */
    uint8_t desired_run = 0;
    
    if (last_motor_run[dev_id] == 0)   /* 当前电机处于停止状态 */
    {
        /* 只有转速绝对值超过启动阈值（500RPM）才允许启动 */
        /* 防止因信号噪声或遥控器微小抖动导致电机误启动 */
        if (set_rpm > MOTOR_START_THRESH || set_rpm < -MOTOR_START_THRESH) {
            desired_run = 1;  /* 进入运行状态 */
        } else {
            desired_run = 0;  /* 保持停止状态 */
            set_rpm = 0;      /* 未达阈值，强制转速为0 */
        }
    }
    else   /* 当前电机处于运行状态 */
    {
        /* 运行中：转速不为0继续运行，转速为0则停止 */
        /* 允许低速运行（<500RPM），不需要重新触发启动阈值 */
        desired_run = (set_rpm != 0) ? 1 : 0;
    }
    
    /* 当前决策的运行状态 */
    uint8_t curr_run_state = desired_run;
    
    /* ========== 阶段1：处理待发送的启动/停止控制指令 ========== */
    /* 根据手册要求，驱动器控制需先写转速再发控制命令 */
    /* 此处通过send_ctrl_pending标志实现两阶段发送 */
    if (send_ctrl_pending[dev_id] == 1)
    {
        if (curr_run_state == 1)
        {
            /* 发送启动命令：REG_CMD_CTRL = CMD_START(0x000F) */
            SendCmd(real_id, REG_CMD_CTRL, CMD_START);
        }
        else
        {
            /* 发送减速停止命令：REG_CMD_CTRL = CMD_STOP_SLOW(0x000E) */
            SendCmd(real_id, REG_CMD_CTRL, CMD_STOP_SLOW);
        }
        send_ctrl_pending[dev_id] = 0;    /* 清除待发送标志 */
        last_motor_run[dev_id] = curr_run_state;  /* 更新状态记录 */
        return;  /* 本轮只发送控制指令，不发送转速 */
    }
    
    /* ========== 阶段2：发送转速设定指令 ========== */
    /* 发送转速：REG_SPEED_SET = set_rpm */
    SendCmd(real_id, REG_SPEED_SET, set_rpm);
    
    /* 判断运行状态是否发生变化 */
    /* 如果状态变化，标记下一轮发送控制指令 */
    if (curr_run_state != last_motor_run[dev_id])
    {
        send_ctrl_pending[dev_id] = 1;  /* 标记下轮发送启动/停止指令 */
    }
}
