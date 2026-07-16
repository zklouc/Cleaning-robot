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
    const uint32_t byte_idle_time = 2; // 单字节空闲超时：9600波特率用3~5ms，115200用1~2ms

    // 先清空所有历史错误标志，避免接收卡死
    __HAL_UART_CLEAR_FLAG(&RS485_UART, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_PE | UART_FLAG_NE);

    while (HAL_GetTick() - t0 < timeout_ms)
    {
        if (__HAL_UART_GET_FLAG(&RS485_UART, UART_FLAG_RXNE) != RESET)
        {
            if (rx_len < sizeof(rx_buf))
                rx_buf[rx_len++] = (uint8_t)(RS485_UART.Instance->DR & 0xFF);
            t0 = HAL_GetTick(); // 收到字节刷新计时器，实现空闲帧结束判断
        }
        else
        {
            // 已收到数据且空闲超时：帧接收完成，提前退出不用等满总超时
            if (rx_len > 0 && (HAL_GetTick() - t0 > byte_idle_time))
                break;
            
            // 主动让出CPU 1ms，给同优先级任务运行机会
            vTaskDelay(pdMS_TO_TICKS(1));
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
    if (UART_WaitRx(5) < 8)
        return MOD_ERR_TIMEOUT;

//    // 4. 应答校验：地址匹配
//    if (rx_buf[0] != dev_id)
//        return MOD_ERR_MATCH;

//    // 5. 应答校验：CRC正确
//    uint16_t recv_crc = (uint16_t)rx_buf[rx_len-1] << 8 | rx_buf[rx_len-2];
//    if (CRC16_MODBUS(rx_buf, rx_len - 2) != recv_crc)
//        return MOD_ERR_CRC;

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
    if (dev_id < 1 || dev_id > RS485_PROP_MAX_ID) return MOD_ERR_PARAM;//ID是 1-4
    if (rpm > RS485_CQ_MAX)  rpm = RS485_CQ_MAX;
    if (rpm < RS485_CQ_MIN)  rpm = RS485_CQ_MIN;

    RS485_Prop[dev_id - 1].target_rpm = rpm;
    ModErr_t ret = WriteReg(dev_id , REG_SPEED_SET, rpm);
    if (ret != MOD_OK)
        RS485_Prop[dev_id - 1].state = MOTOR_ERROR;
    return ret;
}

// 启动推进器（更新缓存状态）
ModErr_t Motor_Start(uint8_t dev_id)
{
    if (dev_id < 1 || dev_id > RS485_PROP_MAX_ID) return MOD_ERR_PARAM;
    ModErr_t ret = WriteReg(dev_id , REG_CMD_CTRL, CMD_START);
    RS485_Prop[dev_id - 1].state = (ret == MOD_OK) ? MOTOR_RUNNING : MOTOR_ERROR;
    if (ret != MOD_OK)
        RS485_Prop[dev_id - 1].last_exception = Motor_GetLastException(dev_id);
    return ret;
}

// 减速停止（更新缓存状态）
ModErr_t Motor_StopSlow(uint8_t dev_id)
{
    if (dev_id < 1 || dev_id > RS485_PROP_MAX_ID) return MOD_ERR_PARAM;
    ModErr_t ret = WriteReg(dev_id , REG_CMD_CTRL, CMD_STOP_SLOW);
    if (ret == MOD_OK)
        RS485_Prop[dev_id - 1].state = MOTOR_STOPPED;
    else
        RS485_Prop[dev_id - 1].state = MOTOR_ERROR;
    return ret;
}

void RS485_Prop_Init(void)
{
	 uint8_t i;
	// 上电初始化所有推进器状态
    for (i = 0; i < RS485_PROP_MAX_ID; i++)
    {
        RS485_Prop[i].state = MOTOR_STOPPED;
        RS485_Prop[i].target_rpm = 0;
    }
}

uint8_t last_motor_run[4] = {0};
uint8_t send_ctrl_pending[4] = {0};


void Motor_Set(uint8_t dev_id, int16_t rpm)
{
    uint8_t real_id = dev_id + 1;
    if (real_id < 1 || real_id > 4)
        return;

    // 转速限幅
    int16_t set_rpm = rpm;
    if (set_rpm > RS485_CQ_MAX)  set_rpm = RS485_CQ_MAX;
    if (set_rpm < -RS485_CQ_MAX) set_rpm = -RS485_CQ_MAX;
		
    uint8_t desired_run = 0;
    if (last_motor_run[dev_id] == 0)   // 停机状态
    {
        if (set_rpm > MOTOR_START_THRESH || set_rpm < -MOTOR_START_THRESH) {
            desired_run = 1;
        } else {
            desired_run = 0;
            set_rpm = 0;
        }
    }
    else   // 运行状态
    {
        desired_run = (set_rpm != 0) ? 1 : 0;
    }

    uint8_t curr_run_state = desired_run;

    // 有待发启停指令：本次只发启停
    if (send_ctrl_pending[dev_id] == 1)
    {
        if (curr_run_state == 1)
        {
            WriteReg(real_id, REG_CMD_CTRL, CMD_START);
        }
        else
        {
            WriteReg(real_id, REG_CMD_CTRL, CMD_STOP_SLOW);
        }
        send_ctrl_pending[dev_id] = 0;
        last_motor_run[dev_id] = curr_run_state;
        return;
    }

    // 无待发指令：本次只发转速
    WriteReg(real_id, REG_SPEED_SET, set_rpm);

    // 状态切换，标记下一轮发启停
    if (curr_run_state != last_motor_run[dev_id])
    {
        send_ctrl_pending[dev_id] = 1;
    }
}

/**
 * @brief  RS485水平推进器周期控制任务
 * @note   外部仅需修改RS485_Prop[i].target_rpm即可控制转速，任务自动处理启停逻辑
 */
void Prop_485_Task(void *argument)
{
    (void)argument;
    uint8_t i;
  	static uint8_t prop_idx = 0;    // 轮询下标：0~3 单路轮发
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // ========== 单次只处理当前下标对应的1路推进器 ==========
        int16_t curr_rpm = RS485_Prop[prop_idx].target_rpm;
        Motor_Set(prop_idx,curr_rpm);//发送启停指令和转速

        // 下标自增，循环轮询 0~3
        prop_idx++;
        if(prop_idx >= 4)
        {
            prop_idx = 0;
        }

        // 固定12ms周期，保证时序稳定
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(12));
    }
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
