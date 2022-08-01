#ifndef __USART_DEBUG_H__
#define __USART_DEBUG_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"
#include "chassis_task.h"





extern UART_HandleTypeDef huart4;


void sendware(void *wareaddr, uint32_t waresize);

#endif
