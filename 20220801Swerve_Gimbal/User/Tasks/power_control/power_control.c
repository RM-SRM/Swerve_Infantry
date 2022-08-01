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
	
	
	if (rpm <= 600)//低速下需要额外抑制 防止超功率
	{
		efficiency = sqrt((float)rpm/4000.0f);
	}
	else if(rpm <=2000)//2000一下一般抑制 随转速上升效率也提升 与转速为对数关系
	{
		efficiency = sqrt((float)rpm/2000.0f);
	}
	else if (rpm >2000)//速度过高也会下降效率 但是不明显 近似为直线
	{
		efficiency = 1-(rpm-2000)*0.00017f;
	}
		
	current_limit = (float)POWERLIMIT_10mW*(float)COEFFCIENT_HIGHSPEED*efficiency/(float)rpm;
	//转矩*转速=输出功率 输出功率/效率=输入功率 
	if(current_limit > 16384)
	{
		current_limit = 16384;
	}
//		current_limit = POWERLIMIT_10mW*COEFFCIENT_HIGHSPEED/rpm; 
	//电机数量为2返回current_limit
	//电机数量为4返回current_limit/2
	return current_limit/2.0f;
}	
