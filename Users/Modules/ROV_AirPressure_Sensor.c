/**
  ******************************************************************************
  * @file    ROV_AirPressure_Sensor.c
  * @author  Liu Y(OUC Fab U+/ROV Team)
  * @brief   气压采集
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#include "ROV_AirPressure_Sensor.h"
#include "Global_Define.h"

uint16_t ADC_Value = 0;
float PRE = 0.0f;

/**
  * @brief 获取压力传感器数据
  *
  * @param 无
	*
  * @retval ADC1原始采样数据
**/
uint16_t AD_GetValue(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 50); 
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		ADC_Value = HAL_ADC_GetValue(&hadc1); 
	}
	//PRE = (((float)ADC_Value / 4095.0f *3.3f) - 0.2f) / 0.0125f;	//实际气压
	return ADC_Value;
}
