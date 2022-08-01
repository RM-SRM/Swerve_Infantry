#ifndef REFEREE_PICTURE_H
#define REFEREE_PICTURE_H


#include "main.h"

#define ROBOT_INTERACTIVE_DATA_SIZE 113

#define ROBOT_INTERACTIVE_DATA							0x0301	

#define ROBOT_INTERACTIVE_DATA_LENGTH					ROBOT_INTERACTIVE_DATA_SIZE

#define DATA_FRAME_HEADER_SOF_DATA						0xA5


/**
* The brief description.
* 数据帧头.
*/
typedef __packed struct
{
	uint8_t 	SOF;											/*固定值*/
	uint16_t 	DATA_LENGTH;							/*数据长度*/
	uint8_t 	SEQ;											/*数据帧中 data 的长度*/			
	uint8_t		CRC_8;										/*CRC_8校验码*/
} ext_data_frame_header_t;


#endif
