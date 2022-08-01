#include "ina226.h"
#include "i2c.h"
//�ô����õ���iic2������Ҫ��iic1������������е�&hi2c2��Ϊ&hi2c1���
void ina226_Init(void)//�趨У׼�Ĵ�����ֵ���Ӷ�ȷ��ILSB��PLSB
{
	uint8_t cal[2]={10,00};//ILSB=1mA
	HAL_I2C_Mem_Write(&hi2c2,0x80,0x05,0x00000001U,cal,2,100);
	osDelay(1);
}

float ina226_read(uint8_t parameter)//ʹ��ʱ����.h�ļ��е���ز�����ȡ��ص�ֵ
{
  uint8_t read_parameter[2]={00,00};
	uint16_t read1;
	float read;
	osDelay(1);
	HAL_I2C_Mem_Read_DMA(&hi2c2,0x80,parameter,0x00000001U,read_parameter,2);
//	HAL_I2C_Mem_Read(&hi2c2,0x80,parameter,0x00000001U,read_parameter,2,100);
	osDelay(1);
	read1=((uint16_t)read_parameter[0]<<8)|((uint16_t)read_parameter[1]);
	if(parameter==vbus)//��ȡ���ߵ�ѹʱ�ֱ���Ϊ1.25mV
	{
		read=(float)read1*0.00125f;
	}
	else if(parameter==I)//��ȡ���ߵ���ʱ�ֱ���Ϊ1mA
	{
		read=(float)read1*1;
	}
	else if(parameter==power)//��ȡ����ʱ�ֱ���Ϊ25mW
	{
		read=(float)read1*25;
	}
	else if(parameter==vr)//��ȡ����ʱ�ֱ���Ϊ2.5uV
	{
		read=(float)read1*2.5f;
	}
	else if(parameter==cali)//�鿴У׼�Ĵ�����ֵ
	{
		read=(float)read1;
	}
	return read;
}

