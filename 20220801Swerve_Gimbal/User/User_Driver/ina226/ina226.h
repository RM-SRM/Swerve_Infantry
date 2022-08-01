#ifndef __INA226_H
#define __INA226_H

#include "FreeRTOS.h"
#include "task.h"

#include "cmsis_os.h"


#include "i2c.h"

//���������������ina226_read(uint8_t parameter)�ж�ȡ��Ӧ��ֵ
//�����λ:vrΪmv,iΪma��powerΪmw��vbusΪv
#define vr 0x01
#define vbus 0x02
#define power 0x03
#define I 0x04
#define cali 0x05
void ina226_Init(void);
float ina226_read(uint8_t parameter);


#endif
