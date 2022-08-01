#include "BTdebug.h"



void BT_debug(debugtype data[], int16_t len)
{
    static uint8_t bt_data[100];
    bt_data[0]=0x03;
    bt_data[1]=~0x03;
//    for(int i=0; i<len; i++)
//    {
//        datatranslate(bt_data+2+(i*sizeof(float)),(uint8_t*)(&data[i]),sizeof(float));
//    }
	memcpy(&bt_data[2], data, len*sizeof(debugtype));
    bt_data[len*sizeof(debugtype)+2]=~0x03;
    bt_data[len*sizeof(debugtype)+3]=0x03;
    HAL_UART_Transmit_DMA(&huart6,bt_data,len*sizeof(debugtype)+4);
}
