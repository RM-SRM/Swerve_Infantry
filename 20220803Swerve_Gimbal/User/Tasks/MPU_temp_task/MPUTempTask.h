#ifndef _MPUTEMPTASK_H
#define _MPUTEMPTASK_H

#include "main.h"
#include "cmsis_os.h"

#include "pid.h"
#include "tim.h"
#include "Sensor_task.h"

#define TEMP_PID_MODE PID_POSITION
#define TEMP_PID_MAX_OUT  6500
#define TEMP_PID_MAX_IOUT 2500
#define TEMP_PID_KP  450.0f
#define TEMP_PID_KI  0.1f
#define TEMP_PID_KD  0

#define TEMP_TASK_CONTROL_TIME  2

typedef struct 
{
	const float *temp;
	float temp_fed;
	float temp_set;
	PID_Regulator_t temp_pid;	
	
}MPU_TEMP;
	
//extern void mpu_temp_task(void const * argument);
const MPU_TEMP *get_temp_control_point(void);




#endif
