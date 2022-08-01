#include "LEDLines_Task.h"

void mouse_press_time(void);

int shoot_press_time = 0;

void LEDLines_Task(void)
{
	for(;;)
	{
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_SET);
         HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
         HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
		 
		 // 8*625==5sµ»¥˝Õ”¬›“«¡„∆Ø 
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
		 osDelay(1000);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
		 osDelay(1000);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
		 osDelay(1000);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
		 osDelay(1000);
		 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_RESET);
		 osDelay(1000);	
        
  }
}

