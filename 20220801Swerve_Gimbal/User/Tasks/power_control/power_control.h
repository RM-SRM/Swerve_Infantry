#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "can_receive.h"
#include "chassis_task.h"

//Ŀǰ30W
#define POWERLIMIT_10mW 					30000

#define COEFFCIENT_HIGHSPEED 			1440.0f

void 		Init_Current_Limit				(void); 
uint16_t Current_Limit_50W (uint16_t t);

#endif //POWER_CONTROL_H
