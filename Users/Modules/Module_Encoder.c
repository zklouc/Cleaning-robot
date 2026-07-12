#include "Module_Encoder.h"

//波特率9600 改了不好使
/* ==================== Modbus命令 ==================== */
static const uint8_t CMD1_READ_VALUE[8]  = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
static const uint8_t CMD1_READ_SPEED[8]  = {0x01, 0x03, 0x00, 0x20, 0x00, 0x02, 0xC5, 0xC1};
static const uint8_t CMD2_READ_VALUE[8]  = {0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x38};
static const uint8_t CMD2_READ_SPEED[8]  = {0x02, 0x03, 0x00, 0x20, 0x00, 0x02, 0xC5, 0xF2};
static const uint8_t CMD1_RESET_ZERO[8]  = {0x01, 0x06, 0x00, 0x08, 0x00, 0x01, 0xC9, 0xC8};
static const uint8_t CMD2_RESET_ZERO[8]  = {0x02, 0x06, 0x00, 0x08, 0x00, 0x01, 0xC9, 0xFB};


uint8_t Encoder_Reset_Flag = 0x00;

/* 全局变量 */
ROV_Encoder_Data ROV_Encoder = {0};
extern osThreadId_t Enc_Data_TaskHandle;

/* 任务栈大小（根据实际情况调整） */
#define ENC_TASK_STACK_SIZE  256

/**
 * @brief 控制 RS485 使能引脚（根据硬件修改引脚）
 * @param mode 1: 发送模式; 0: 接收模式
 */
void RS485_SetMode(uint8_t mode)
{
    if (mode) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);   // 发送模式
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); // 接收模式
    }
}

/**
 * @brief  RS485 完整事务：发送命令 + 等待回复（带超时）
 * @param  cmd        命令数组指针（8字节）
 * @param  addr       期望的从机地址（0x01 或 0x02）
 * @param  func       期望的功能码（0x03 或 0x06）
 * @param  replyBuf   存放回复数据的缓冲区（至少8字节）
 * @param  timeoutMs  超时时间（毫秒）
 * @retval HAL_OK 成功收到有效回复，否则 HAL_TIMEOUT 或 HAL_ERROR
 */
HAL_StatusTypeDef RS485_Exchange(const uint8_t *cmd, uint8_t addr, uint8_t func,
                                        uint8_t *replyBuf, uint32_t timeoutMs)
{
    HAL_StatusTypeDef ret;
    Frame_TypeDef frame;

    /* ---- 第1步：清空队列，去除旧数据 ---- */
    xQueueReset(UsartPort[ENC_UART_ID].rxQueue);

    /* ---- 第2步：切换为发送模式 ---- */
    RS485_SetMode(1);

    /* ---- 第3步：发送命令 ---- */
    ret = HAL_UART_Transmit(UsartPort[ENC_UART_ID].huart, (uint8_t*)cmd, 8, timeoutMs);
    if (ret != HAL_OK) {
        RS485_SetMode(0);   // 发送失败立即恢复接收模式
        return ret;
    }

    /* ---- 第4步：等待发送完成（TC标志） ---- */
    while (__HAL_UART_GET_FLAG(&RS485_UART, UART_FLAG_TC) == RESET);

    /* ---- 第5步：切换为接收模式 ---- */
    RS485_SetMode(0);

    /* ---- 第6步：循环等待有效回复（带超时） ---- */
    TickType_t startTick = xTaskGetTickCount();
    TickType_t timeoutTicks = pdMS_TO_TICKS(timeoutMs);

    while ((xTaskGetTickCount() - startTick) < timeoutTicks) {
        // 从队列取帧，每次等待 10ms（如果无数据则重复尝试直到超时）
        if (xQueueReceive(UsartPort[COMPASS_UART_ID].rxQueue, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            // 检查帧长度至少8字节，且地址和功能码匹配
            if (frame.len >= 8 && frame.data[0] == addr && frame.data[1] == func) {
                // 可选：CRC校验（确保数据完整性）
                uint16_t recv_crc = ((uint16_t)frame.data[7] << 8) | frame.data[6];
                uint16_t calc_crc = CRC16_MODBUS(frame.data, 6);
                if (recv_crc == calc_crc) {
                    memcpy(replyBuf, frame.data, 8);
                    return HAL_OK;
                }
            }
        }
    }

    /* 超时，未收到正确回复 */
    return HAL_TIMEOUT;
}

/**
 * @brief  读取编码器的通用函数
 * @param  cmd       命令指针
 * @param  addr      从机地址
 * @param  pDest     存放结果的指针（16位）
 * @param  timeoutMs 超时时间
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef ReadEncoderValue(const uint8_t *cmd, uint8_t addr, uint16_t *pDest, uint32_t timeoutMs)
{
    uint8_t reply[8];
    HAL_StatusTypeDef ret = RS485_Exchange(cmd, addr, 0x03, reply, timeoutMs);
    if (ret == HAL_OK) {
        // Modbus 数据格式：reply[3] 为高字节，reply[4] 为低字节
        *pDest = ((uint16_t)reply[3] << 8) | reply[4];
        return HAL_OK;
    }
    return ret;
}

/* ===================== 编码器任务主体 ===================== */
void Encoder_Task(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(200);   // 200ms 周期

    while (1) {
        /* ---- 读取编码器1 原始值 ---- */
        if (ReadEncoderValue(CMD1_READ_VALUE, 0x01, &ROV_Encoder.RawValue1, 100) != HAL_OK) {
            ROV_Encoder.ErrCnt++;
        }

        /* ---- 读取编码器1 速度 ---- */
        if (ReadEncoderValue(CMD1_READ_SPEED, 0x01, &ROV_Encoder.Speed1, 100) != HAL_OK) {
            ROV_Encoder.ErrCnt++;
        }

        /* ---- 读取编码器2 原始值 ---- */
        if (ReadEncoderValue(CMD2_READ_VALUE, 0x02, &ROV_Encoder.RawValue2, 100) != HAL_OK) {
            ROV_Encoder.ErrCnt++;
        }

        /* ---- 读取编码器2 速度 ---- */
        if (ReadEncoderValue(CMD2_READ_SPEED, 0x02, &ROV_Encoder.Speed2, 100) != HAL_OK) {
            ROV_Encoder.ErrCnt++;
        }

        /* ---- 按固定周期延时 ---- */
        vTaskDelayUntil(&lastWakeTime, periodTicks);
    }
}

//EncoderState_t currentState = STATE_SEND_CMD1;


//void ROV_Send_CMD(void)
//{
//	  if(Encoder_Reset_Flag==0x00)
//		{
//			switch(currentState)
//			{
//					case STATE_SEND_CMD1:
//							HAL_UART_Transmit(&huart5, (uint8_t*)CMD1_READ_VALUE, 8, 100);
//							currentState = STATE_SEND_CMD2;
//							break;
//							
//					case STATE_SEND_CMD2:
//							HAL_UART_Transmit(&huart5, (uint8_t*)CMD1_READ_SPEED, 8, 100);
//							currentState = STATE_SEND_CMD3;
//							break;
//							
//					case STATE_SEND_CMD3:
//							HAL_UART_Transmit(&huart5, (uint8_t*)CMD2_READ_VALUE, 8, 100);
//							currentState = STATE_SEND_CMD4;
//							break;
//							
//					case STATE_SEND_CMD4:
//							HAL_UART_Transmit(&huart5, (uint8_t*)CMD2_READ_SPEED, 8, 100);
//							currentState = STATE_SEND_CMD1;
//							break;
//			}	
//		}
//		else if(Encoder_Reset_Flag == 0x01)
//		{ 

//			HAL_UART_Transmit(&huart5, (uint8_t*)CMD1_RESET_ZERO, 8, 100);  

//			Encoder_Reset_Flag = 0x02;		
//		}
//		else if(Encoder_Reset_Flag == 0x02)
//		{

//			HAL_UART_Transmit(&huart5, (uint8_t*)CMD2_RESET_ZERO, 8, 100);  

//			Encoder_Reset_Flag = 0x00;		
//		}	

//}



//void ROV_Encoders_Process(uint8_t *UART5_RxBuf)
//{
//			// 根据当前状态解析数据
//			switch(currentState)
//			{
//					case STATE_SEND_CMD2:  // 刚发送完CMD1，现在收到的是CMD1的回复
//							if(UART5_RxBuf[0] == 0x01 && UART5_RxBuf[1] == 0x03)
//							{
//									uint32_t raw = ((uint16_t)UART5_RxBuf[5] << 8) | UART5_RxBuf[6];
//									ROV_Encoder.RawValue1 = raw & 0xFFFF;
//							}
//							break;
//							
//					case STATE_SEND_CMD3:  // 刚发送完CMD2，现在收到的是CMD2的回复
//							if(UART5_RxBuf[0] == 0x01 && UART5_RxBuf[1] == 0x03)
//							{
//									uint32_t speed = ((uint16_t)UART5_RxBuf[5] << 8) | UART5_RxBuf[6];
//									ROV_Encoder.Speed1 = speed & 0xFFFF;
//							}
//							break;
//							
//					case STATE_SEND_CMD4:  // 刚发送完CMD3，现在收到的是CMD3的回复
//							if(UART5_RxBuf[0] == 0x02 && UART5_RxBuf[1] == 0x03)
//							{
//									uint32_t raw = ((uint16_t)UART5_RxBuf[5] << 8) | UART5_RxBuf[6];
//									ROV_Encoder.RawValue2 = raw & 0xFFFF;
//							}
//							break;
//							
//					case STATE_SEND_CMD1:  // 刚发送完CMD4，现在收到的是CMD4的回复
//							if(UART5_RxBuf[0] == 0x02 && UART5_RxBuf[1] == 0x03)
//							{
//									uint32_t speed = ((uint16_t)UART5_RxBuf[5] << 8) | UART5_RxBuf[6];
//									ROV_Encoder.Speed2 = speed & 0xFFFF;
//							}
//							break;
//			}		
//}
