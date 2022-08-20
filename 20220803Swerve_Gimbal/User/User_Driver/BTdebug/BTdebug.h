#ifndef _BTDEBUG_H
#define _BTDEBUG_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"


typedef float debugtype;

extern void BT_debug(debugtype data[], int16_t len);
#endif

