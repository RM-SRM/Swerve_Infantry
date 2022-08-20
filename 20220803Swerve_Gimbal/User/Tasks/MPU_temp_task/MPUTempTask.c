#include "MPUTempTask.h"

static MPU_TEMP mpu_temp;

static void Temp_Control_Init(MPU_TEMP *temp_init);
static void Temp_Feedback_Update(MPU_TEMP *temp_feedback);
static void Temp_Set_Conteol(MPU_TEMP *temp_set);
static void Temp_PID(MPU_TEMP *temp_pid);


void mpu_temp_task(void const * argument)
{

	  uint32_t waitetime;
	  Temp_Control_Init(&mpu_temp);
	  waitetime = xTaskGetTickCount();
	
  for(;;)
	
	{
		Temp_Feedback_Update(&mpu_temp);
		Temp_Set_Conteol(&mpu_temp);
		Temp_PID(&mpu_temp);
		
		TIM3->CCR2 = (uint16_t)mpu_temp.temp_pid.out;
		
	  osDelayUntil(&waitetime, TEMP_TASK_CONTROL_TIME);
	}
}

void Temp_Control_Init(MPU_TEMP *temp_init)
{
    if(temp_init == NULL)
		{
		  return;
		}
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    //温度传感器PID初始化
		temp_init->temp = get_MPU6500_Temperate_Data_Point(); 
		PID_Init(&mpu_temp.temp_pid,TEMP_PID_MODE,TEMP_PID_MAX_OUT,TEMP_PID_MAX_IOUT,TEMP_PID_KP,TEMP_PID_KI,TEMP_PID_KD);

}
void Temp_Feedback_Update(MPU_TEMP *temp_feedback)
{
    if(temp_feedback == NULL)
		{
		  return;
		}
    temp_feedback->temp_fed = *temp_feedback->temp;

}

void Temp_Set_Conteol(MPU_TEMP *temp_set)
{
    if(temp_set ==NULL)
		{
		  return;
		}
		temp_set->temp_set = 50.0f;

}
void Temp_PID(MPU_TEMP *temp_pid)
{
    if(temp_pid == NULL)
		{
		  return;
		}
		PID_Calculate(&temp_pid->temp_pid,temp_pid->temp_fed,temp_pid->temp_set);
		
		if(temp_pid->temp_pid.out < 0)
		{
		  temp_pid->temp_pid.out = 0;
		}
}

const MPU_TEMP *get_temp_control_point(void)
{
    return &mpu_temp;
}
