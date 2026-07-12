/* Includes ------------------------------------------------------------------*/
#include "Module_CAN.h"
/* 变量声明 ------------------------------------------------------------------*/
static CAN_TxHeaderTypeDef	TxHeader;      //发送
static CAN_RxHeaderTypeDef	RxHeader;      //接收

/**
  * @brief CAN滤波器配置，配置为全部接收
  *
  * @param 无
	*
  * @retval 无
**/
void CAN_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*配置CAN滤波器*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
	
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    while(1)
	  {
			
	  }
  }
	sFilterConfig.FilterBank = 14;

  /*启动CAN外设*/
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    while(1)
	  {
			
	  }
  }
	
  /*激活CAN接收中断通知*/
	uint32_t activeITs = CAN_IT_ERROR_WARNING|CAN_IT_ERROR_PASSIVE|CAN_IT_LAST_ERROR_CODE|CAN_IT_ERROR;
  activeITs = CAN_IT_ERROR_WARNING|CAN_IT_ERROR_PASSIVE|CAN_IT_LAST_ERROR_CODE| CAN_IT_ERROR |CAN_IT_RX_FIFO0_MSG_PENDING;
  if (HAL_CAN_ActivateNotification(&hcan1, activeITs) != HAL_OK) 
	{
    Error_Handler();
  }

  /*配置CAN发送句柄*/
  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x0501EF70;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
}


/**
	* @brief  CAN发送推进器控制指令
	*
	* @param  hcan					CAN句柄指针
  * @param  msg        		待发送数据的缓冲区首地址
  * @param  len        		待发送数据长度
  * @param  CAN_EXT_ID 		29位扩展标识符
	*
  * @retval 0  发送成功
  *         1  发送请求入队失败
  *         2  等待发送完成超时
**/
uint8_t Can_Prop_Send(CAN_HandleTypeDef *hcan, uint8_t *msg, uint8_t len, uint32_t CAN_EXT_ID)
{
    uint8_t i = 0,cnt = 0;
		uint32_t TxMailbox;
		uint8_t message[8];
    TxHeader.StdId = 0X00;        //标准标识符
    TxHeader.ExtId = CAN_EXT_ID;  //扩展标识符(29位)
    TxHeader.IDE = CAN_ID_EXT;    //使用扩展帧
    TxHeader.RTR = CAN_RTR_DATA;  //数据帧
    TxHeader.DLC = len;        
	//可不拷贝	
    for(i = 0; i < len; i++)
    {
			message[i]=msg[i];
		}
    
		while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) 
		{
			if(++cnt >= 200)
			{
				cnt = 0;
				return 2;//超时
			}
		}
		if(HAL_CAN_AddTxMessage(hcan, &TxHeader, message, &TxMailbox) != HAL_OK)//发送
		{
			return 1;//报错
		}
		return 0;//正常
}





/**
  * @brief CAN接收数据处理
  *       
  *      
  *
  * @param data:推进器反馈数据，Rx_Header:CAN接收结构体
  * @retval 无
  */
void CAN_Data_Pro(uint8_t *data,CAN_RxHeaderTypeDef *Rx_Header)
{
		for(uint8_t j = 0;j < 4;j++)
		{
			uint8_t i= Vertical_Prop_ID[j];
		  {
		    Prop_CAN[i].Prop_life = Prop_CAN[i].Prop_life>3?0:Prop_CAN[i].Prop_life;
			  if(Rx_Header->ExtId == (i+0x050170E0))
			  {
					 Prop_CAN[i].Prop_Rx_Temperature = data[1];
					 Prop_CAN[i].Prop_Rx_Current = data[2];
					 Prop_CAN[i].Prop_Rx_Volte = (((uint16_t)data[4])<<8) + (uint16_t)data[3];
					 Prop_CAN[i].Prop_Rx_Warning = data[5];
					 Prop_CAN[i].Prop_Rx_RPM = (((uint16_t)data[7])<<8) + (uint16_t)data[6];
			  }
		  }
		}
}


/**
	* @brief CAN1总线上的发送任务
  *
  * @param argument 		任务入口参数
	*
  * @retval 无
**/
void CAN1_SendData(void *argument)
{
	/* 记录起始节拍 */
	static TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
  for(;;)
  {
		for (uint8_t Device_ID = V_Prop_RF; Device_ID <= H_Prop_LF; Device_ID++)
		{
			/* 0:垂向上锁---6:水平上锁 */
			if (Device_ID == V_Prop_RF) osMutexAcquire(Vertical_MutexHandle, osWaitForever);
			if (Device_ID == H_Prop_RF) osMutexAcquire(Horizontal_MutexHandle, osWaitForever);
			
			/* 数据发送 */
			uint8_t buf[8] = {0};
			buf[3] = Motor_Device[Device_ID].CQ_Current;
			buf[4] = Motor_Device[Device_ID].CQ_Current >> 8;
			buf[7] = 0x70 + Device_ID;
			
			Motor_Device[Device_ID].Result = Can_Prop_Send(&hcan1, buf, 8, Prop_Tx_BaseID + Device_ID);
			osDelay(2);
			
			/* 5:垂向解锁---9:水平解锁 */
			if (Device_ID == V_Prop_LF) osMutexRelease(Vertical_MutexHandle);
			if (Device_ID == H_Prop_LF) osMutexRelease(Horizontal_MutexHandle);
		}
		
		/* 任务总时长固定30ms */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));
  }
}


/**
	* @brief  CAN接收FIFO0消息挂起中断回调函数
	*
	* @param  hcan					CAN句柄指针
	*
  * @retval 无
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == hcan1.Instance)
	{
		static uint8_t buf[8];
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, buf) == HAL_OK)			//获得接收到的数据头和数据
		{
			if(life_Calibration==0)
			{
				life = buf[0];
				life_Calibration = 1;
			}
			CAN_Data_Pro(buf,&RxHeader);
			HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);	//重新使能FIFO0接收中断

		}
	}
}
