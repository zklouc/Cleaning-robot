/* Includes ------------------------------------------------------------------*/
#include "Module_CAN_Prop.h"
/* 变量声明 ------------------------------------------------------------------*/
static CAN_TxHeaderTypeDef	TxHeader;      //发送
static CAN_RxHeaderTypeDef	RxHeader;      //接收


CAN_Prop_CtrlTypeDef CAN_Prop[4];

uint8_t Vertical_Prop_ID[4] = {7,6,2,1};

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
  TxHeader.ExtId = CAN_Prop_Tx_BaseID;
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
uint8_t CAN1_Send_Msg(uint8_t* msg,uint8_t len,uint32_t CAN_EXT_ID)
{	
    uint8_t i=0,cnt = 0;
		uint32_t TxMailbox;
		uint8_t message[8];
    TxHeader.StdId=0X00;        //标准标识符
    TxHeader.ExtId=CAN_EXT_ID;  //扩展标识符(29位)
    TxHeader.IDE=CAN_ID_EXT;    //使用扩展帧
    TxHeader.RTR=CAN_RTR_DATA;  //数据帧
    TxHeader.DLC=len;                
    for(i=0;i<len;i++)
    {
			message[i]=msg[i];
		}
    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, message, &TxMailbox) != HAL_OK)//发送
		{
			return 1;
		}
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) 
		{
			cnt ++;
			if(cnt >= 20)
			{
				cnt = 0;
				return 2;
			}

		}
		return 0;
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
			uint8_t dev_id = Vertical_Prop_ID[j];
		  {
		    CAN_Prop[j].Prop_life = CAN_Prop[j].Prop_life>3?0:CAN_Prop[j].Prop_life;
			  if(Rx_Header->ExtId == (dev_id + 0x050170E0))
			  {
					 CAN_Prop[j].Prop_Rx_Temperature = data[1];
					 CAN_Prop[j].Prop_Rx_Current = data[2];
					 CAN_Prop[j].Prop_Rx_Voltage = (((uint16_t)data[4])<<8) + (uint16_t)data[3];
					 CAN_Prop[j].Prop_Rx_Warning = data[5];
					 CAN_Prop[j].Prop_Rx_RPM = (((uint16_t)data[7])<<8) + (uint16_t)data[6];
//					 CAN_Prop[j].Prop_Tx_Curren_Range = 0X64;
			  }
		  }
		}
}


/**
	* @brief CAN1总线上的推进器发送任务
  * @note  使用CAN_Prop结构体组包，ID、序号与接收端完全对齐
  * @param argument 任务入口参数
	* @retval 无
**/
void CAN1_SendData(void *argument)
{
	static TickType_t xLastWakeTime;
	static uint8_t prop_idx = 0;    // 轮询下标：0~3 单路轮发
  static uint8_t snap_buf[4][8];  // 整轮快照缓冲区：4路*8字节，保证同一轮数值完全一致
  static uint8_t snap_devid[4];   // 对应设备ID快照
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
        // 每轮第一次（下标0）：一次性读取全部4路数据到本地快照，保证原子性
        if (prop_idx == 0)
        {
            osMutexAcquire(Vertical_MutexHandle, osWaitForever);
            for (uint8_t i = 0; i < 4; i++)
            {
                snap_devid[i] = Vertical_Prop_ID[i];
                snap_buf[i][0] = CAN_Prop[i].Prop_Tx_Config;
                snap_buf[i][1] = CAN_Prop[i].Prop_Tx_Mode;
                snap_buf[i][2] = CAN_Prop[i].Prop_Tx_Direction;
                snap_buf[i][3] = (uint8_t)(CAN_Prop[i].Prop_Tx_Ctl & 0xFF);
                snap_buf[i][4] = (uint8_t)(CAN_Prop[i].Prop_Tx_Ctl >> 8);
                snap_buf[i][5] = CAN_Prop[i].Prop_Tx_Curren_Range;
                snap_buf[i][6] = CAN_Prop[i].Prop_Tx_Start_Time;
                snap_buf[i][7] = 0x00;
            }
            osMutexRelease(Vertical_MutexHandle);
        }

        // 锁外执行CAN发送，不占用共享资源锁
        Prop_Ctrol.Manual_V_RES[prop_idx] = CAN1_Send_Msg(
            snap_buf[prop_idx], 
            8, 
            snap_devid[prop_idx] + 0x0501EF70
        );

        // 下标轮询
        prop_idx++;
        if(prop_idx >= 4)
        {
            prop_idx = 0;
        }

        // 固定12.5ms等效间隔，4路一轮总周期≈48ms，与控制任务适配
		    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(12));
	}
}
//void CAN1_SendData(void *argument)
//{
//	static TickType_t xLastWakeTime;
//	static uint8_t prop_idx = 0;   // 轮询下标：0~3 单路轮发
//	xLastWakeTime = xTaskGetTickCount();
//	uint8_t tx_buf[8];
//	uint8_t dev_id;

//	for(;;)
//	{
//			// 1. 互斥锁仅保护共享数据读取、组包，拷贝到局部变量后立即解锁
//			osMutexAcquire(Vertical_MutexHandle, osWaitForever);
//			dev_id = Vertical_Prop_ID[prop_idx];

//			// 从 CAN_Prop[prop_idx] 结构体拆解数据到局部发送缓冲区
//			tx_buf[0]  = CAN_Prop[prop_idx].Prop_Tx_Config;
//			tx_buf[1]  = CAN_Prop[prop_idx].Prop_Tx_Mode;
//			tx_buf[2]  = CAN_Prop[prop_idx].Prop_Tx_Direction;
//			tx_buf[3]  = (uint8_t)(CAN_Prop[prop_idx].Prop_Tx_Ctl & 0xFF);
//			tx_buf[4]  = (uint8_t)(CAN_Prop[prop_idx].Prop_Tx_Ctl >> 8);
//			tx_buf[5]  = CAN_Prop[prop_idx].Prop_Tx_Curren_Range;  // 0x64;
//			tx_buf[6]  = CAN_Prop[prop_idx].Prop_Tx_Start_Time;
//			tx_buf[7]  = 0x00;

//			// 2. 锁外执行CAN发送，不占用共享资源锁
//			Prop_Ctrol.Manual_V_RES[prop_idx] = CAN1_Send_Msg(tx_buf, 8, dev_id + 0x0501EF70);
//			osMutexRelease(Vertical_MutexHandle);
//		  // 3. 下标轮询
//		  prop_idx++;
//			if(prop_idx >= 4)
//			{
//				prop_idx = 0;
//			}

//			// 4. 固定12ms单路间隔，4路一轮总周期≈48ms，与50ms控制任务适配
//			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(12));
//	}
//}

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
			CAN_Data_Pro(buf,&RxHeader);
			HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);	//重新使能FIFO0接收中断

		}
	}
}






/**@brief      有轴推进器进行初始化
*  @param[in]  无
*  @return     无
*  @details    控制量复位
*  @attention  调用初始化调用
*/
void CAN_Prop_Init(void)
{
	uint8_t j= 0;
	for(j=0;j<4;j++)
	{
		CAN_Prop[j].Prop_Tx_Config = 0;
		CAN_Prop[j].Prop_Tx_Mode = 0x01;
		CAN_Prop[j].Prop_Tx_Direction = 0;
		CAN_Prop[j].Prop_Tx_Ctl = 0x00;
		CAN_Prop[j].Prop_Tx_Curren_Range = 0x00;
		CAN_Prop[j].Prop_Tx_Start_Time = 0x00;
		CAN_Prop[j].Prop_EXTID = CAN_Prop_Tx_BaseID + Vertical_Prop_ID[j];
	}
}


