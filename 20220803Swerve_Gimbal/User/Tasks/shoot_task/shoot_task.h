#ifndef _SHOOT_TASK_H
#define _SHOOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "remote.h"
#include "pid.h"
#include "tim.h"







//�������� ����ȫ����д
#define SHOOT_TASK_CONTROL_TIME 1
#define SINGLESHOOTECDCOUNT 	36864						//���Եó�����תһ��ļ��� ��Ӧһ��
#define FULLECD 				8192
#define ECDDVALE_MAX 			FULLECD/2					//��1000Hz�Ĳ���Ƶ���� ת�ǶȲ����ܳ���0.5Ȧ
#define BLOCKTIMEMAX			20
#define REVERSETIME				200
#define FRONTTIME				30
#define ROLLINGDIRECTION		CLOCKWISE
#define FRICTIONDIRECTION		CLOCKWISE
#define MM17HEAT				10
#define MM42HEAT				100
#define LOCATION_TIME			4000
#define LOCATION_WAIT_TIME		800
#define MM17SHOOTER
#ifndef MM17SHOOTER
#define MM42SHOOTER
#endif

//ö�������� ����e+��д
enum eSHOOTBLOCKSTATE{NOT_BLOCK = 0,BLOCK
};
enum ebuttonState{NOT_PRESS,SHORT_PRESS,LONG_PRESS,TRIGGERSHOOTING
};
enum eSHOOTSTAUES{NOT_FIRE = 0, SINGLESHOOT, TRISHOOT, CONTINUESHOOT
};
enum eFRICTIONSTAUES{CLOSE,OPEN};
enum elocationState{LOCDONE = 0, LOCSTART
};

//ROLLING �ٶȻ� PID�����Լ� PID���������������
#define ROLLING_SPEED_PID_MODE_POSITION 
#ifdef ROLLING_SPEED_PID_MODE_POSITION
#define ROLLING_SPEED_PID_MODE PID_POSITION
#define ROLLING_SPEED_PID_MAX_OUT 4000.0f
#define ROLLING_SPEED_PID_MAX_IOUT 3000.0f

#define ROLLING_SPEED_PID_KP 12.0f
#define ROLLING_SPEED_PID_KI 0.0f
#define ROLLING_SPEED_PID_KD 0.0f
#endif

//ROLLING ECDλ�û� PID�����Լ� PID���������������
#define ROLLING_POSITION_PID_MODE_POSITION 
#ifdef ROLLING_POSITION_PID_MODE_POSITION
#define ROLLING_POSITION_PID_MODE PID_POSITION
#define ROLLING_POSITION_PID_MAX_OUT  36 * 190.0f //190.0f
#define ROLLING_POSITION_PID_MAX_IOUT 36 * 100.0f

#define ROLLING_POSITION_PID_KP 0.20f
#define ROLLING_POSITION_PID_KI 0.0f
#define ROLLING_POSITION_PID_KD 0.0f
#endif

//Friction Ħ�����ٶȻ� PID�����Լ� PID���������������
//c620���������16384
#define FRICTION_POSITION_PID_MODE_POSITION 
#ifdef FRICTION_POSITION_PID_MODE_POSITION
#define FRICTION_POSITION_PID_MODE PID_POSITION
#define FRICTION_POSITION_PID_MAX_OUT  10000.0f
#define FRICTION_POSITION_PID_MAX_IOUT 10000.0f/3.0f

#define FRICTION_POSITION_PID_KP 11.0f
#define FRICTION_POSITION_PID_KI 0.008f
#define FRICTION_POSITION_PID_KD 0.0f
#endif

#define SHOOT_LEFT_TIME_MAX 150



//�ṹ�嶨�� ��������_t
//�������ٶ�pid�ṹ��
typedef struct
{
	const motor_ecd_measure_t *rollingMotorMeasure;
	
	float speed;
	float speed_set;
	
	PID_Regulator_t speed_pid;
	
} Rolling_Motor_Speed_t;

//������λ��pid�ṹ��
typedef struct
{
	Rolling_Motor_Speed_t rollingSpeed;
	
	int64_t EcdPosition_set;
	
	PID_Regulator_t position_pid;
	
} Rolling_Motor_Position_t;

//Ħ���ֽṹ��
typedef struct
{
	const motor_measure_t *rollingMotorMeasure;
	
	float speed;
	float speed_set;
	
	PID_Regulator_t speed_pid;
	
} Friction_Motor_Speed_t;

//�����pid�ṹ��
typedef struct
{
	Rolling_Motor_Position_t rollingPosition;
	Friction_Motor_Speed_t frictionSpeed[2];
	
	//Ħ���������ٶ��趨
	uint16_t fricSpeedRPM;
	enum eFRICTIONSTAUES 	fricStaues;
	//δ����Ҫ����ϵͳ��ָ���Ħ���ֿ���
	uint8_t shoot_frie;
	uint16_t bullet_speed;
	uint16_t cooling_Rate;
	uint16_t cooling_Limit;
	uint16_t cooling_Data_Now;
  int16_t shoot_left;
	uint16_t shoot_speed_max;
	//����������״̬
	uint16_t block_time;
	uint32_t last_ecdset;
	uint16_t button_press_time;
	//״̬�����ñ���
	enum ebuttonState 		buttonState;
	enum eSHOOTSTAUES 		fireStatus;
	enum eSHOOTBLOCKSTATE 	blockStage;
	enum ebuttonState		lastButtonState;
	enum elocationState		locationState;	
} Shoot_Control_t;

#endif


const Shoot_Control_t *get_shoot_motor_control_point(void);

extern Shoot_Control_t shoot_control; 
extern volatile int32_t WaitTimeForShoot;
