#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "can_receive.h"
#include "remote.h"
#include "pid.h"
#include "gimbal_task.h"
#include "referee.h"
#include "SupercapTask.h"
#include "arm_math.h"


//PID
#define CHASSIS_PID_MODE          PID_POSITION
#define CHASSIS_PID_MAX_OUT       8000
#define CHASSIS_PID_MAX_IOUT      2400
#define CHASSIS_PID_KP            9      //4
#define CHASSIS_PID_KI            0.3   //0.5f      //1
#define CHASSIS_PID_KD            0    //0.08f      //0



#define CHASSIS_ROTATE_PID_MODE          PID_POSITION
#define CHASSIS_ROTATE_PID_MAX_OUT       6000
#define CHASSIS_ROTATE_PID_MAX_IOUT      2000
#define CHASSIS_ROTATE_PID_KP            2000      //4
#define CHASSIS_ROTATE_PID_KI            0.0f      //1
#define CHASSIS_ROTATE_PID_KD            0.0f      //0
//


#define GIMBAL_TASK_CONTROL_TIME  1



//struct define
typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	float accel;
	float speed;
	float speed_set;
	
  PID_Regulator_t speedrpm_pid;
	PID_Regulator_t speed_pid;

	
} Chassis_Motor_t;

typedef struct
{
	float M_global_local[4];
	int16_t global_front;
	int16_t global_left;
	float local_front;
	float local_left;
	int16_t rotate;
	
} Dirction;


typedef struct
{
	const Gimbal_Motor_t *chassis_yaw_motor;	
	const Gimbal_Motor_t *chassis_pitch_motor;
	
	
	const RC_ctrl_t *chassis_RC;     
	Chassis_Motor_t chassis_motor[4];
    //底盘跟随PID
	PID_Regulator_t rotate_pid;
    
	Dirction dir;
	
}Chassis_Control_t;






extern void chassis_Task(void const * argument);
const Chassis_Control_t *get_motor_control_point(void);

#endif
