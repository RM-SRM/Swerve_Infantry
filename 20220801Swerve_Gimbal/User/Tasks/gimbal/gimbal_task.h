#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "remote.h"
#include "Sensor_task.h"
#include "pid.h"
#include "auto_aim_task.h"
#include "shoot_task.h"



//gimbaltask控制周期
#define GIMBAL_TASK_CONTROL_TIME 1
//云台电机初始值（中值）电机重新安装之后，需要重新设定此参数
#define PITCH_OFFSET_ECD 3330
#define YAW_OFFSET_ECD 1334

//云台电机一周期及其一半
#define FULL_RANGE 8192
#define HALF_RANGE 4096

//将云台一周期8192的精度转化为一周期2π
#define Motor_Ecd_to_Rad 0.000766990394f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAE_SPEED_PID_MODE_POSITION 1
#ifdef YAE_SPEED_PID_MODE_DELTA
#define YAW_SPEED_PID_MODE PID_DELTA
#define YAW_SPEED_PID_MAX_OUT 20000.0f
#define YAW_SPEED_PID_MAX_IOUT 2000.0f

#define YAW_SPEED_PID_KP 4000.0f
#define YAW_SPEED_PID_KI 18000.0f
#define YAW_SPEED_PID_KD 40.0f

#elif YAE_SPEED_PID_MODE_POSITION
#define YAW_SPEED_PID_MODE PID_POSITION


//yaw 速度环 PID参数以及 PID最大输出，积分输出
#ifdef YAE_SPEED_PID_MODE_POSITION
#define YAW_SPEED_PID_MODE PID_POSITION
#define YAW_SPEED_PID_MAX_OUT 25000.0f
#define YAW_SPEED_PID_MAX_IOUT 4000.0f //6000
#define YAW_SPEED_PID_KP 7500.0f   //6000
#define YAW_SPEED_PID_KI 30.0f     //5.0f //30.0f         //20     

#define YAW_SPEED_PID_KD 0
#endif


//yaw 角度环 PID参数 外环位置式PID 
#define YAE_ANGLE_PID_MODE_POSITION
#ifdef YAE_ANGLE_PID_MODE_POSITION
#define YAW_ANGLE_PID_MODE PID_POSITION

#define YAW_ANGLE_PID_MAX_OUT 12.0f
#define YAW_ANGLE_PID_MAX_IOUT 0.8f
#define YAW_ANGLE_PID_KP 13.0f
#define YAW_ANGLE_PID_KI 0   //0.5f
#define YAW_ANGLE_PID_KD 0       //-3.0f
#endif


//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_MODE_POSITION
#ifdef PITCH_SPEED_PID_MODE_POSITION
#define PITCH_SPEED_PID_MAX_OUT 27000.0f
#define PITCH_SPEED_PID_MAX_IOUT 7000.0f   
#define PITCH_SPEED_PID_MODE PID_POSITION

#define PITCH_SPEED_PID_KP 8000.0f
#define PITCH_SPEED_PID_KI 30.0f //60.0f
#define PITCH_SPEED_PID_KD 0

#endif

//pitch 角度环 PID参数 外环位置式PID 
#define PITCH_ANGLE_PID_MODE_POSITION
#ifdef PITCH_ANGLE_PID_MODE_POSITION
#define PITCH_ANGLE_PID_MODE PID_POSITION
#define PITCH_ANGLE_PID_MAX_OUT 12.10f
#define PITCH_ANGLE_PID_MAX_IOUT 0.8f

#define PITCH_ANGLE_PID_KP 12.0f
#define PITCH_ANGLE_PID_KI 0
#define PITCH_ANGLE_PID_KD 0
#endif

#define AIM_MISSED        -1


typedef struct
{
	const motor_measure_t *gimbal_motor_measure;
	
	float gyro;
	float gyro_set;
	float absolute_angle;
	float absolute_angle_set;
	float relative_angle;
	float relative_angle_set;
	float offset_AbToRe;

	PID_Regulator_t speed_pid;
	PID_Regulator_t angle_pid;

	uint16_t offset_ecd;
	const Target_t *target;

} Gimbal_Motor_t;

typedef struct
{
	const RC_ctrl_t *gimbal_rc_ctrl;
    const float	*gimbal_INT_angle_point;
    const float *gimbal_INT_gyro_point;
	Gimbal_Motor_t pitch_motor;
	Gimbal_Motor_t yaw_motor;
	
} Gimbal_Control_t;


extern void gimbal_task(void const * argument);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
#endif


 extern void gimbal_task(void const * argument);


extern void gimbal_task(void const * argument);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);

#endif

