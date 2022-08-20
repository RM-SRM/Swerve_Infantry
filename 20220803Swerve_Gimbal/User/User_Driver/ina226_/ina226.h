#ifndef INA226_H 
#define INA226_H

#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "i2c.h"

#define Ina226Address 		0x80
#define ConfigRegister		0x00
#define ShuntVoltRegister	0x01
#define BusVoltRegister 	0x02
#define PowerRegister 		0x03
#define CurrentRegister		0x04
#define CalRegister				0x05
#define DelayTime					1

typedef struct
{
		uint16_t shunt;
		uint16_t bus;
		uint16_t current;
		uint16_t power;
} ina226_data;


const ina226_data *	Get_Ina226_Point		(void);
void 								in226Init						(I2C_HandleTypeDef *hi2cx);
uint16_t 						GetPower_10mW 			(I2C_HandleTypeDef *hi2cx);
uint16_t 						GetVote 						(I2C_HandleTypeDef *hi2cx);
uint16_t 						GetCurrent 					(I2C_HandleTypeDef *hi2cx);
uint16_t 						GetShuntVolt 				(I2C_HandleTypeDef *hi2cx);

void ina226_Init(void);
#endif // INA226_H 
