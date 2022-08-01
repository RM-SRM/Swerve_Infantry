#include "power_control.h"
#include <stdlib.h>
#include <math.h>


static const motor_measure_t* measure_motor[4]; 
static uint16_t current_limit;

void Init_Current_Limit()
{
	int i;
	for (i=0;i<4;i++)
	{
		measure_motor[i] = get_Chassis_Motor_Measure_Point(i);
	}
}

uint16_t Current_Limit_50W(uint16_t t)
{
	uint16_t rpm;
	rpm = abs(measure_motor[t]->speed_rpm); 
	if (rpm <1){rpm = 1;}
	float efficiency;
	
	
	if (rpm <= 600)//��������Ҫ�������� ��ֹ������
	{
		efficiency = sqrt((float)rpm/4000.0f);
	}
	else if(rpm <=2000)//2000һ��һ������ ��ת������Ч��Ҳ���� ��ת��Ϊ������ϵ
	{
		efficiency = sqrt((float)rpm/2000.0f);
	}
	else if (rpm >2000)//�ٶȹ���Ҳ���½�Ч�� ���ǲ����� ����Ϊֱ��
	{
		efficiency = 1-(rpm-2000)*0.00017f;
	}
		
	current_limit = (float)POWERLIMIT_10mW*(float)COEFFCIENT_HIGHSPEED*efficiency/(float)rpm;
	//ת��*ת��=������� �������/Ч��=���빦�� 
	if(current_limit > 16384)
	{
		current_limit = 16384;
	}
//		current_limit = POWERLIMIT_10mW*COEFFCIENT_HIGHSPEED/rpm; 
	//�������Ϊ2����current_limit
	//�������Ϊ4����current_limit/2
	return current_limit/2.0f;
}	
