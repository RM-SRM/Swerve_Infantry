#ifndef __INA226_H
#define __INA226_H

#include "FreeRTOS.h"
#include "task.h"

#include "cmsis_os.h"


#include "i2c.h"

//以下五个参数填入ina226_read(uint8_t parameter)中读取相应的值
//输出单位:vr为mv,i为ma，power为mw，vbus为v
#define vr 0x01
#define vbus 0x02
#define power 0x03
#define I 0x04
#define cali 0x05
void ina226_Init(void);
float ina226_read(uint8_t parameter);


#endif
