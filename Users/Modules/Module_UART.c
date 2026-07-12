/* Includes ------------------------------------------------------------------*/
#include "Module_UART.h"
/* 变量声明 ------------------------------------------------------------------*/
bool RS485_Mode;
uint16_t JoyStick[4] = {1500,1500,1500,1500};
UsartPort_TypeDef UsartPort[6] = 
{
	{ .huart = &huart1 },   	// 第 0 路：USART1---TTL---主通信
	{ .huart = &huart2 },   	// 第 1 路：USART2---TTL---水温水深
	{ .huart = &huart3 },   	// 第 2 路：USART3---485---滚刷角度编码器
	{ .huart = &huart4 },   	// 第 3 路：UART4----TTL---姿态传感器
	{ .huart = &huart5 },   	// 第 4 路：UART5----TTL---空闲
	{ .huart = &huart6 }    	// 第 5 路：USART6---232---空闲
};
uint8_t test = 0;


/**
  * @brief 帧排序对齐
  *
  * @param *input_buf 	待对齐数据
	* @param *output_buf 	对齐数据
	* @param len 					帧长度
	* @param start_h 			帧头高字节
	* @param start_l 			帧头低字节
	*
  * @retval 无
**/
void Frame_Align(uint8_t *input_buf, uint8_t *output_buf, uint8_t len, uint8_t start_h, uint8_t start_l)
{
	int8_t pos = -1;
	
	for (int i = 0; i < len - 1; i++)
	{
		if (input_buf[i] == start_h && input_buf[i + 1] == start_l)
		{
			pos = i;
			break;
		}
	}
	
	if (pos == -1)
	{
		memset(output_buf, 0, len);
		return;
	}
	else
	{
		for (uint8_t i = 0; i < len; i++)
		{
			uint8_t src_index = (pos + i) % len;
			output_buf[i] = input_buf[src_index];
		}
		return;
	}
}


/**
  * @brief 六路串口初始化
  *
  * @param 无
	*
  * @retval 无
**/
void User_UART_Init(void)
{
	for (int i = 0; i < 6; i++)
	{
		/* 创建队列 */
		UsartPort[i].rxQueue = xQueueCreate(2, sizeof(Frame_TypeDef));
		if (UsartPort[i].rxQueue == NULL)
		{
			while(1);
    }
		
		/* 初始化缓冲区 */
		memset(UsartPort[i].rxBuf, 0, sizeof(UsartPort[i].rxBuf));
		
		/* 关闭半传输中断 */
		__HAL_DMA_DISABLE_IT(UsartPort[i].huart->hdmarx, DMA_IT_HT);
		
		/* 启动DMA接收 */
		if (HAL_UARTEx_ReceiveToIdle_DMA(UsartPort[i].huart, UsartPort[i].rxBuf, HOST_RECEIVE_LEN) != HAL_OK)
		{
			while(1);
		}
	}
}


/**
  * @brief 串口中断中央回调
  *
  * @param 	*huart				触发中断的UART句柄
	*
  * @retval 无
**/
void USER_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
	/* 只负责IDLE中断 */
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET)
			return;

	/* 清除IDLE标志 */
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	
	/* 计算真实收到字节数 */
	uint16_t real_len = HOST_RECEIVE_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

	/* 串口号映射 */
	int idx = IDX(huart);
	UsartPort_TypeDef *PortNow = &UsartPort[idx];
	
	/* 帧重排序 */
	switch (idx)
	{
		case 0:
			Frame_Align(PortNow->rxBuf, PortNow->rxBuf_Align, HOST_RECEIVE_LEN, HOST_START_H, HOST_START_L);
			break;
		case 1:
			Frame_Align(PortNow->rxBuf, PortNow->rxBuf_Align, HOST_RECEIVE_LEN, PRESS_START_H, PRESS_START_L);
			break;
		case 2:
			Frame_Align(PortNow->rxBuf, PortNow->rxBuf_Align, HOST_RECEIVE_LEN, 0x01, 0x03);
			break;
		case 3:
			Frame_Align(PortNow->rxBuf, PortNow->rxBuf_Align, HOST_RECEIVE_LEN, 0x68, 0x0D);
			break;
		case 4:
			Frame_Align(PortNow->rxBuf, PortNow->rxBuf_Align, HOST_RECEIVE_LEN, 0x68, 0x0D);
			break;
		default: break;
	}
	
	/* 封装帧并传进队列*/
	Frame_TypeDef frame_IT = { .len = HOST_RECEIVE_LEN };
	memcpy(frame_IT.data, PortNow->rxBuf_Align, HOST_RECEIVE_LEN);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	/* 修改后的 ISR 发送部分 */
	BaseType_t ret;
	ret = xQueueSendFromISR(PortNow->rxQueue, &frame_IT, &xHigherPriorityTaskWoken);

	/* 打断点在这里：如果 ret != pdTRUE，说明队列满了 */
	if (ret != pdTRUE)
	{
		test++;
			// 这里说明任务来不及处理，或者队列深度太浅
			// 可以在这里增加一个错误计数变量方便观察
	}
	
//	xQueueSendFromISR(PortNow->rxQueue, &frame_IT, &xHigherPriorityTaskWoken);
	
	/* 任务切换 */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
	* @brief 串口发送完成回调
  *
  * @param 	huart					触发中断的UART句柄
	*
  * @retval 无
**/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	/* 快速知道是哪路串口 */
  int idx = IDX(huart);
	
	if (idx == PROP_485_UART_ID)
	{
		/* 485控制引脚置低电平为接收模式 */
		HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port, RS485_CTRL_Pin, GPIO_PIN_RESET);
	}
}
