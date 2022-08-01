#ifndef _SWERVE_TASK_H
#define _SWERVE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "can_receive.h"
#include "remote.h"
#include "pid.h"

#include "arm_math.h"



//swervetask控制周期
#define SWERVE_TASK_CONTROL_TIME 1

//6020电机一周期及其一半
#define FULL_RANGE 8192
#define HALF_RANGE 4096

//底盘跟随正方向
#define YAW_OFFSET_ECD 5807

//将6020电机周期8192的精度转化为一周期2π
#define Motor_Ecd_to_Rad 0.000766990394f

//PID
#define SWERVE_POSITIONPID_MODE          PID_POSITION
#define SWERVE_POSITIONPID_MAX_OUT       1600.0f
#define SWERVE_POSITIONPID_MAX_IOUT      600.0f
#define SWERVE_POSITIONPID_KP            3.0f   // 0.09f// 9.0f
#define SWERVE_POSITIONPID_KI            0.0f    //0.0f//12.0f
#define SWERVE_POSITIONPID_KD            0.0f    // 0.0f//5.0f



#define SWERVE_SPEEDPID_MODE             PID_POSITION
#define SWERVE_SPEEDPID_MAX_OUT          11000  //25000
#define SWERVE_SPEEDPID_MAX_IOUT         2000  //7000
#define SWERVE_SPEEDPID_KP               10    //1.05f
#define SWERVE_SPEEDPID_KI               0.005   //0.001f
#define SWERVE_SPEEDPID_KD               0   //0.0f

#define CHASSIS_ROTATE_PID_MODE          PID_POSITION
#define CHASSIS_ROTATE_PID_MAX_OUT       3500
#define CHASSIS_ROTATE_PID_MAX_IOUT      500
#define CHASSIS_ROTATE_PID_KP            3500      //4
#define CHASSIS_ROTATE_PID_KI            0.0f      //1
#define CHASSIS_ROTATE_PID_KD            0.0f      //0

#define SWREVE0_OFFSET_ECD      1400
#define SWREVE1_OFFSET_ECD      6858
#define SWREVE2_OFFSET_ECD      1978
#define SWREVE3_OFFSET_ECD      4647





//struct define
typedef struct
{
	const motor_measure_t *server_measure;
	float gyro;
	float gyro_set;
	float relative_angle;
	float relative_angle_set;
	float last_relative_angle_set;
	double relative_angle_algorithm;
  uint16_t ecd_fdb;
	int16_t  ecd_set;
	double   ecd_algorithm;
  uint16_t offset_ecd;
	
	PID_Regulator_t speed_pid;
	PID_Regulator_t position_pid;

	


} Swerve_Motor_t;


typedef struct
{
	const Swerve_direction_t  *get_dir;
	float M_global_local[4];
	float local_front;
	float local_left;
	int16_t rotate;
	
} Dirction;

typedef struct
{
	int16_t chassismotorspeed_algorithm	[4];
	int8_t direction_change[4];
	Swerve_Motor_t swerve_motor[4];
	Dirction dir;
	PID_Regulator_t rotate_pid;
	
	const motor_measure_t *yaw_measure;

	const RC_ctrl_t *swerve_RC; 
  
	float chassis_yaw_relative_angle;
	int16_t yaw_ecderror;
	float angle45;
	float angle90;
	float angle180;
	float angle270;
	
}Swerve_Control_t;


extern volatile uint8_t Swerve_start_flag;

extern const Swerve_Control_t *get_swerve_motor_point(void);

extern void swerve_Task(void const * argument);


#endif
