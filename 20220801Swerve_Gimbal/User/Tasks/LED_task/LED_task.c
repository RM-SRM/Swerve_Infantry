/**
  *******************************************************
  * @file       LED_task.h
  * @brief      ∞Â‘ÿLED…¡À∏√ø√Î2¥Œ
  * @note       

  @verbatim    
  ==============================================================================

  ==============================================================================
  @endverbatim
  *******************************************************
  */

#include "LED_task.h"

int gimbal_count = 1;

void LED_Task(void const * argument)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
	for(;;)
	{			
		gimbal_count = 1; 
		HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
		osDelay(1000);
		gimbal_count = -1; 
		HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
		osDelay(1000);	
	}
}

