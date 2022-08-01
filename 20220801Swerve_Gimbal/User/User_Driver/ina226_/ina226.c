#include "ina226.h"
static ina226_data ina226;

const ina226_data *Get_Ina226_Point()
{
    return &ina226;
}

void in226Init(I2C_HandleTypeDef *hi2cx)
{
//	uint16_t Clear =0x0000;
//	uint16_t ClearConfig = 0x2741;
	uint16_t InitConfig = 0xFF41; /*	0100-
																		000/不取平均值-
																		111/8.244ms总线电压-
																		111/8.244ms采样电流
																		111/连续采样
																*/
	uint16_t Calibration = 0x000A;
	
//	HAL_I2C_Mem_Write(&hi2c2,0x80,0X00,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&ClearConfig,sizeof(ClearConfig),0XFF);
//	osDelay(1);	
//	
//	HAL_I2C_Mem_Write(&hi2c2,0x80,0X05,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Clear,sizeof(Clear),0xFF);
//	osDelay(1);

	HAL_I2C_Mem_Write(hi2cx,Ina226Address,ConfigRegister,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&InitConfig,sizeof(InitConfig),1000);
	osDelay(DelayTime);

	HAL_I2C_Mem_Write(hi2cx,Ina226Address,CalRegister,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Calibration,sizeof(Calibration),1000);
	osDelay(DelayTime);

}

uint16_t GetPower_10mW (I2C_HandleTypeDef *hi2cx)
{
	uint16_t Power;
	
	HAL_I2C_Mem_Read_DMA(hi2cx,Ina226Address,PowerRegister,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Power,sizeof(Power));
	osDelay(DelayTime);
	
	Power = Power >> 8 | Power << 8;
	Power = Power*25/10;
	ina226.power = Power;
	return Power;
}

uint16_t GetVote (I2C_HandleTypeDef *hi2cx)
{
	uint16_t Vote;
	
	HAL_I2C_Mem_Read_DMA(&hi2c2,Ina226Address,BusVoltRegister,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Vote,sizeof(Vote));
	osDelay(DelayTime);
	
	Vote = Vote >> 8 | Vote << 8;
	Vote = Vote*1.25;
	ina226.bus = Vote;
	return Vote;
}

uint16_t GetCurrent (I2C_HandleTypeDef *hi2cx)
{
	uint16_t Current;
	
	HAL_I2C_Mem_Read_DMA(&hi2c2,Ina226Address,CurrentRegister,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Current,sizeof(Current));
	osDelay(DelayTime);
	
	Current = Current >> 8 | Current << 8;
	ina226.current = Current; 
	return Current;
}

uint16_t GetShuntVolt (I2C_HandleTypeDef *hi2cx)
{
	uint16_t ShuntVolt;
	
	HAL_I2C_Mem_Read_DMA(&hi2c2,Ina226Address,ShuntVoltRegister,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&ShuntVolt,sizeof(ShuntVolt));
	osDelay(DelayTime);
	
	ShuntVolt = ShuntVolt >> 8 | ShuntVolt << 8;
	ina226.shunt = ShuntVolt;
	return ShuntVolt;
}
