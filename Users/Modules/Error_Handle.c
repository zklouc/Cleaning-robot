#include "Error_Handle.h"


/**
  * @brief 串口错误中断回调函数
  *
	* @param  *huart					串口句柄
	*
  * @retval 无
**/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	/* 快速知道是哪路串口 */
	int idx = IDX(huart);
	
//	if (idx == COMPASS_UART_ID)
//	{
//		/* 获取并清除所有错误标志 */
//		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))		//溢出错误
//		{
//			__HAL_UART_CLEAR_OREFLAG(huart);
//		}
//		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) 		//噪声错误
//		{
//				__HAL_UART_CLEAR_NEFLAG(huart);
//		}
//		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) 		//帧错误
//		{
//				__HAL_UART_CLEAR_FEFLAG(huart);
//		}
		/* 无论哪种错误都直接重启DMA */
		HAL_UARTEx_ReceiveToIdle_DMA(UsartPort[idx].huart, UsartPort[idx].rxBuf, HOST_RECEIVE_LEN);
//	}
}
