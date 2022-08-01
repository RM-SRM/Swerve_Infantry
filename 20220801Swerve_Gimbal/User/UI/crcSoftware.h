/** 
* @file		crcSoftware.h 
* @brief 	软CRC校验的头文件. 
* @author   抄大疆 
* @date    	date 
* @version  	V
* @par 		History:          
*  		 version: author, date, desc\n 
*/  
#include "main.h"
/*************************************************************CRC循环校验部分函数*************************************************************/
unsigned char 	Get_CRC8_Check_Sum				(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int 	Verify_CRC8_Check_Sum			(unsigned char *pchMessage, unsigned int dwLength);
void 			Append_CRC8_Check_Sum			(unsigned char *pchMessage, unsigned int dwLength);

uint16_t 		Get_CRC16_Check_Sum				(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t 		Verify_CRC16_Check_Sum			(uint8_t *pchMessage, uint32_t dwLength);
void 			Append_CRC16_Check_Sum			(uint8_t * pchMessage,uint32_t dwLength);
/*************************************************************CRC循环校验部分函数*************************************************************/
