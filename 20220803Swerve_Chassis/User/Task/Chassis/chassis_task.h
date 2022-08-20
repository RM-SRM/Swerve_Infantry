#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "swerve_task.h"
#include "can_receive.h"
#include "remote.h"
#include "pid.h"
#include "arm_math.h"


////定义键鼠wsad普通速度及shift加速后速度
//#define SPEED_LOW 3000
//#define SPEED_HIGH 5600
//#define SPEED_ROLE 3000
//#define SPEED_FLY 13000



////定义飞坡后两轮速度比例
//#define FlySlopeRatio 1.1f
////飞坡模式遥控器转速度比例定义
//#define FlyRCtoSpeedRatioX_Y 11
//#define FlyRCtoSpeedRatioZ 6


//#define Key_ctrl  1


//PID
#define CHASSIS_PID_MODE          PID_POSITION
#define CHASSIS_PID_MAX_OUT       6000
#define CHASSIS_PID_MAX_IOUT      1600
#define CHASSIS_PID_KP            9     //4
#define CHASSIS_PID_KI            0.03   //0.5f      //1
#define CHASSIS_PID_KD            0    //0.08f      //0




#define CHASSIS_TASK_CONTROL_TIME  1



//struct define
typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	float accel;
	float speed;
	float speed_set;
	float speed_algorithm;
  PID_Regulator_t speedrpm_pid;
	PID_Regulator_t speed_pid;

	
} Chassis_Motor_t;




typedef struct
{
	
	const RC_ctrl_t *chassis_RC;     
	const Swerve_Control_t *chassis_swerve;
	Chassis_Motor_t chassis_motor[4];
    //底盘跟随PID
	PID_Regulator_t rotate_pid;
 
	const uint8_t *chassis_powermax;
	const int16_t *chassis_buffer;
	
	const supercap_measure_t *cap;
}Chassis_Control_t;



extern Chassis_Control_t chassis_control;


extern void chassis_Task(void const * argument);
const Chassis_Control_t *get_motor_control_point(void);

#endif
