#ifndef _REFEREE_USART_TASK_H
#define _REFEREE_USART_TASK_H


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "usart.h"




#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);
extern volatile uint8_t finish_flag;

void uart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
