#ifndef MODULE_UART_H
#define MODULE_UART_H

#include <Global_Define.h>

#define HOST_RECEIVE_LEN   		 	 		 128u
#define HOST_FEEDBACK_LEN 					 128u
#define IDX(huart) \
    ((huart)->Instance == USART1 ? 0 : \
     (huart)->Instance == USART2 ? 1 : \
     (huart)->Instance == USART3 ? 2 : \
     (huart)->Instance == UART4  ? 3 : \
     (huart)->Instance == UART5  ? 4 : 5)


/* 各外设串口顺序定义 & 对应的实际MCU上串口号 */
typedef enum
{
  HOST_UART_ID = 0,													//USART1--主通信
  DPH_UART_ID,															//USART2--水温水深
	ENC_UART_ID,													  	//USART3--旋转编码器
	COMPASS_UART_ID,													//UART4---电子罗盘
	PROP_485_UART_ID,													//UART5---485驱动推进器
	RS232_UART_ID															//USART6--空闲
}UART_IDTypeDef;

/* 每路串口结构体的完整描述 */
typedef struct
{
    UART_HandleTypeDef 							 *huart;    //huart句柄
    uint8_t  				rxBuf[HOST_RECEIVE_LEN];    //接收缓冲
		uint8_t   rxBuf_Align[HOST_RECEIVE_LEN];		//排序完的接收帧
    QueueHandle_t 									rxQueue;    //与任务通信的队列
} UsartPort_TypeDef;

/* 串口接收队列传输数据包 */
typedef struct
{
  uint16_t len;
	uint8_t  data[HOST_RECEIVE_LEN];
} Frame_TypeDef;


extern UsartPort_TypeDef UsartPort[6];
extern bool RS485_Mode;
extern uint16_t JoyStick[4];


void User_UART_Init(void);
void USER_UART_IdleCpltCallback(UART_HandleTypeDef *huart);

#endif
