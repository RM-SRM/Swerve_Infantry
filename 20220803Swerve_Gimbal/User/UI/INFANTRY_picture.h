#ifndef INFANTRY_PICTURE_H
#define INFANTRY_PICTURE_H

#include "main.h"
#include "crcSoftware.h"
#include <string.h>
#include "referee_picture.h"
#include <stdio.h>
#include "referee.h"
/* 字符长度 */
#define DEL_LEN                          6+2
#define D1_LEN                          6+15
#define D2_LEN                          6+30
#define D5_LEN                          6+75
#define D7_LEN                         6+105
#define DCHAR_LEN                       6+45

/* 内容ID */
#define DEL_ID                        0x0100
#define D1_ID                         0x0101
#define D2_ID                         0x0102
#define D5_ID                         0x0103
#define D7_ID                         0x0104
#define DCHAR_ID                      0x0110

#define CMD_LEN                            2
#define HEADER_LEN                         5
#define CRC_LEN                            2

#define delay                        osDelay
#define Append_CRC8_Check_Sum_p      append_CRC8_check_sum
#define Append_CRC16_Check_Sum_p     append_CRC16_check_sum

/*位置*/
#define F0_Y      540   //第0线y轴位置
#define F100_Y    520   //第100线y轴位置
#define F200_Y    490   //第200线y轴位置
#define F300_Y    450   //第300线y轴位置
#define F400_Y    350   //第400线y轴位置

#define F0_LEN    200   //第0线长度
#define F100_LEN  180   //第100线长度
#define F200_LEN  140   //第200线长度
#define F300_LEN  60    //第300线长度
#define F400_LEN  30    //第400线长度

#define sc_X      500   //超级电容位置
#define sc_Y      700
#define st_X      500   //指示位置
#define st_Y      650   

typedef __packed struct
{
	uint16_t 																Cmd_Id;
	ext_student_interactive_header_data_t			Student_Interactive_Header_Data;
}	picture_data_head;

void static_picture(UART_HandleTypeDef  *huart,uint16_t robot_id);
void flash_picture_init(UART_HandleTypeDef  *huart,uint16_t robot_id);
void flash_picture(UART_HandleTypeDef  *huart,uint16_t robot_id,uint16_t supercap,uint8_t keyboard);
#endif
