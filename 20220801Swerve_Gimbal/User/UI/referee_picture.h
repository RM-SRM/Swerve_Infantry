#ifndef REFEREE_PICTURE_H
#define REFEREE_PICTURE_H


#include "main.h"

#define ROBOT_INTERACTIVE_DATA_SIZE 113

#define ROBOT_INTERACTIVE_DATA							0x0301	

#define ROBOT_INTERACTIVE_DATA_LENGTH					ROBOT_INTERACTIVE_DATA_SIZE

#define DATA_FRAME_HEADER_SOF_DATA						0xA5


/**
* The brief description.
* ����֡ͷ.
*/
typedef __packed struct
{
	uint8_t 	SOF;											/*�̶�ֵ*/
	uint16_t 	DATA_LENGTH;							/*���ݳ���*/
	uint8_t 	SEQ;											/*����֡�� data �ĳ���*/			
	uint8_t		CRC_8;										/*CRC_8У����*/
} ext_data_frame_header_t;


#endif
