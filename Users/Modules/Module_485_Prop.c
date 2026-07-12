#include "Module_485_Prop.h"

uint8_t Horizontal_Prop_ID[4] = {1,2,3,4};

RS485_Prop_CtrlTypeDef RS485_Prop[RS485_PROP_MAX_ID];

// 通信缓冲区（精简为最小必要长度）
static uint8_t tx_buf[16];
static uint8_t rx_buf[16];
static uint16_t rx_len;


// ===================== 底层工具函数 =====================

// 等待串口接收，返回接收字节数
static uint16_t UART_WaitRx(uint32_t timeout_ms)
{
    rx_len = 0;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms)
    {
        if (__HAL_UART_GET_FLAG(&RS485_UART, UART_FLAG_RXNE) != RESET)
        {
            if (rx_len < sizeof(rx_buf))
                rx_buf[rx_len++] = (uint8_t)(RS485_UART.Instance->DR & 0xFF);
            t0 = HAL_GetTick();
        }
    }
    return rx_len;
}

// ===================== 核心读写寄存器函数 =====================

/**
 * @brief  写单个保持寄存器（0x10功能码）+ 完整应答校验
 * @param  dev_id 从站地址（1~4）
 * @param  reg 寄存器地址
 * @param  value 写入值
 * @return ModErr_t 通信错误码
 */
ModErr_t WriteReg(uint8_t dev_id, uint16_t reg, int16_t value)
{
    if (dev_id < 1 || dev_id > RS485_PROP_MAX_ID)
        return MOD_ERR_PARAM;

    // 1. 打包写帧
    uint8_t tx_len = 0;
    tx_buf[tx_len++] = dev_id;
    tx_buf[tx_len++] = 0x10;
    tx_buf[tx_len++] = (reg >> 8) & 0xFF;
    tx_buf[tx_len++] = reg & 0xFF;
    tx_buf[tx_len++] = 0x00;
    tx_buf[tx_len++] = 0x01;
    tx_buf[tx_len++] = 0x02;
    tx_buf[tx_len++] = (value >> 8) & 0xFF;
    tx_buf[tx_len++] = value & 0xFF;
    uint16_t crc = CRC16_MODBUS(tx_buf, tx_len);
    tx_buf[tx_len++] = crc & 0xFF;
    tx_buf[tx_len++] = (crc >> 8) & 0xFF;

    // 2. 发送帧
    if (HAL_UART_Transmit(&RS485_UART, tx_buf, tx_len, 50) != HAL_OK)
        return MOD_ERR_TX;
    while (__HAL_UART_GET_FLAG(&RS485_UART, UART_FLAG_TC) == RESET);

    // 3. 接收应答（写单个寄存器应答固定8字节）
    if (UART_WaitRx(100) < 8)
        return MOD_ERR_TIMEOUT;

    // 4. 应答校验：地址匹配
    if (rx_buf[0] != dev_id)
        return MOD_ERR_MATCH;

    // 5. 应答校验：CRC正确
    uint16_t recv_crc = (uint16_t)rx_buf[rx_len-1] << 8 | rx_buf[rx_len-2];
    if (CRC16_MODBUS(rx_buf, rx_len - 2) != recv_crc)
        return MOD_ERR_CRC;

    // 6. 应答校验：驱动器返回异常
    if (rx_buf[1] & 0x80)
    {
        RS485_Prop[dev_id - 1].error_code = rx_buf[2];
        return MOD_ERR_EXCEPTION;
    }

    // 7. 应答校验：功能码+寄存器地址+数量 与发送一致（就是你说的数组比对逻辑）
    if (rx_buf[1] != 0x10 || 
        ((uint16_t)rx_buf[2] << 8 | rx_buf[3]) != reg ||
        ((uint16_t)rx_buf[4] << 8 | rx_buf[5]) != 0x0001)
        return MOD_ERR_MATCH;

    return MOD_OK;
}

/**
 * @brief  读多个保持寄存器（0x03功能码）+ 完整应答校验
 * @param  dev_id 从站地址（1~4）
 * @param  reg 起始寄存器地址
 * @param  cnt 寄存器数量
 * @param  data_out 输出数据缓冲区（长度至少cnt*2）
 * @return ModErr_t 通信错误码
 */
ModErr_t ReadReg(uint8_t dev_id, uint16_t reg, uint16_t cnt, uint8_t *data_out)
{
    if (dev_id < 1 || dev_id > RS485_PROP_MAX_ID || data_out == NULL || cnt == 0)
        return MOD_ERR_PARAM;

    // 1. 打包读帧
    uint8_t tx_len = 0;
    tx_buf[tx_len++] = dev_id;
    tx_buf[tx_len++] = 0x03;
    tx_buf[tx_len++] = (reg >> 8) & 0xFF;
    tx_buf[tx_len++] = reg & 0xFF;
    tx_buf[tx_len++] = (cnt >> 8) & 0xFF;
    tx_buf[tx_len++] = cnt & 0xFF;
    uint16_t crc = CRC16_MODBUS(tx_buf, tx_len);
    tx_buf[tx_len++] = crc & 0xFF;
    tx_buf[tx_len++] = (crc >> 8) & 0xFF;

    // 2. 发送帧
    if (HAL_UART_Transmit(&RS485_UART, tx_buf, tx_len, 50) != HAL_OK)
        return MOD_ERR_TX;
    while (__HAL_UART_GET_FLAG(&RS485_UART, UART_FLAG_TC) == RESET);

    // 3. 接收应答（最小长度=地址1+功能码1+字节数1+数据N+CRC2）
    uint16_t min_len = 5 + cnt * 2;
    if (UART_WaitRx(100) < min_len)
        return MOD_ERR_TIMEOUT;

    // 4. 地址校验
    if (rx_buf[0] != dev_id)
        return MOD_ERR_MATCH;

    // 5. CRC校验
    uint16_t recv_crc = (uint16_t)rx_buf[rx_len-1] << 8 | rx_buf[rx_len-2];
    if (CRC16_MODBUS(rx_buf, rx_len - 2) != recv_crc)
        return MOD_ERR_CRC;

    // 6. 异常应答校验
    if (rx_buf[1] & 0x80)
    {
        RS485_Prop[dev_id - 1].last_exception = rx_buf[2];
        return MOD_ERR_EXCEPTION;
    }

    // 7. 功能码+数据长度校验
    if (rx_buf[1] != 0x03 || rx_buf[2] != cnt * 2)
        return MOD_ERR_MATCH;

    // 8. 拷贝有效数据
    memcpy(data_out, &rx_buf[3], cnt * 2);
    return MOD_OK;
}

// ===================== 上层电机控制接口 =====================

// 设置转速（带限幅，自动更新缓存）
ModErr_t Motor_SetSpeed(uint8_t dev_id, int16_t rpm)
{
    if (dev_id >= RS485_PROP_MAX_ID) return MOD_ERR_PARAM;
    if (rpm > MOTOR_SPEED_MAX)  rpm = MOTOR_SPEED_MAX;
    if (rpm < -MOTOR_SPEED_MAX) rpm = -MOTOR_SPEED_MAX;

    RS485_Prop[dev_id - 1].target_rpm = rpm;
    ModErr_t ret = WriteReg(dev_id , REG_SPEED_SET, rpm);
    if (ret != MOD_OK)
        RS485_Prop[dev_id - 1].state = MOTOR_ERROR;
    return ret;
}

// 启动推进器（更新缓存状态）
ModErr_t Motor_Start(uint8_t dev_id)
{
    if (dev_id >= RS485_PROP_MAX_ID) return MOD_ERR_PARAM;
    ModErr_t ret = WriteReg(dev_id , REG_CMD_CTRL, CMD_START);
    RS485_Prop[dev_id - 1].state = (ret == MOD_OK) ? MOTOR_RUNNING : MOTOR_ERROR;
    if (ret != MOD_OK)
        RS485_Prop[dev_id - 1].last_exception = Motor_GetLastException(dev_id);
    return ret;
}

// 减速停止（更新缓存状态）
ModErr_t Motor_StopSlow(uint8_t dev_id)
{
    if (dev_id >= RS485_PROP_MAX_ID) return MOD_ERR_PARAM;
    ModErr_t ret = WriteReg(dev_id , REG_CMD_CTRL, CMD_STOP_SLOW);
    if (ret == MOD_OK)
        RS485_Prop[dev_id - 1].state = MOTOR_STOPPED;
    else
        RS485_Prop[dev_id - 1].state = MOTOR_ERROR;
    return ret;
}

// ===================== 状态读取接口 =====================

// 读取完整状态并更新缓存
ModErr_t Motor_GetStatus(uint8_t dev_id)
{
    if (dev_id >= RS485_PROP_MAX_ID)
        return MOD_ERR_PARAM;

    ModErr_t ret;
    uint8_t buf[4];

    // 读实际转速
    ret = ReadReg(dev_id , REG_SPEED_ACTUAL, 1, buf);
    if (ret != MOD_OK) return ret;
    RS485_Prop[dev_id - 1].actual_rpm = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);

    // 读实际电流（4字节浮点）
    ret = ReadReg(dev_id , REG_CURRENT_ACTUAL, 2, buf);
    if (ret != MOD_OK) return ret;
    uint32_t f_raw = (uint32_t)buf[0] << 24 | (uint32_t)buf[1] << 16 | (uint32_t)buf[2] << 8 | buf[3];
    RS485_Prop[dev_id - 1].actual_current = *(float *)&f_raw;

    // 读报警码
    ret = ReadReg(dev_id , REG_ERR_CODE, 1, buf);
    if (ret != MOD_OK) return ret;
    RS485_Prop[dev_id - 1].error_code = (uint16_t)buf[0] << 8 | buf[1];

    return MOD_OK;
}

// 获取缓存状态（无通信，直接返回上次结果）
RS485_Prop_CtrlTypeDef *Motor_GetLastStatus(uint8_t dev_id)
{
    if (dev_id >= RS485_PROP_MAX_ID) return NULL;
    return &RS485_Prop[dev_id - 1];
}

// 获取驱动器返回的Modbus异常码（启动失败时调用排查）
uint8_t Motor_GetLastException(uint8_t dev_id)
{
    if (dev_id >= RS485_PROP_MAX_ID) return 0;
    return RS485_Prop[dev_id - 1].error_code;
}
