#include "ina226.h"
#include "i2c.h"
//该代码用的是iic2，若需要用iic1，则将下面代码中的&hi2c2改为&hi2c1便可
void ina226_Init(void)//设定校准寄存器的值，从而确定ILSB和PLSB
{
	uint8_t cal[2]={10,00};//ILSB=1mA
	HAL_I2C_Mem_Write(&hi2c2,0x80,0x05,0x00000001U,cal,2,100);
	osDelay(1);
}

float ina226_read(uint8_t parameter)//使用时填入.h文件中的相关参数读取相关的值
{
  uint8_t read_parameter[2]={00,00};
	uint16_t read1;
	float read;
	osDelay(1);
	HAL_I2C_Mem_Read_DMA(&hi2c2,0x80,parameter,0x00000001U,read_parameter,2);
//	HAL_I2C_Mem_Read(&hi2c2,0x80,parameter,0x00000001U,read_parameter,2,100);
	osDelay(1);
	read1=((uint16_t)read_parameter[0]<<8)|((uint16_t)read_parameter[1]);
	if(parameter==vbus)//读取总线电压时分辨率为1.25mV
	{
		read=(float)read1*0.00125f;
	}
	else if(parameter==I)//读取总线电流时分辨率为1mA
	{
		read=(float)read1*1;
	}
	else if(parameter==power)//读取功率时分辨率为25mW
	{
		read=(float)read1*25;
	}
	else if(parameter==vr)//读取功率时分辨率为2.5uV
	{
		read=(float)read1*2.5f;
	}
	else if(parameter==cali)//查看校准寄存器的值
	{
		read=(float)read1;
	}
	return read;
}

