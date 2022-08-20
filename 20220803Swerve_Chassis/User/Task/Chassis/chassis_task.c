/*
  ---               ---
 |   |-------------|   |
 | 0 |-------------| 3 |
  ---               ---
  | |               | |
  | |       ^       | |
  | |      ^ ^      | |
  | |     ^   ^     | |
  | |     |   |     | |
  | |     |   |     | |
  | |     -----     | |
  | |               | |
  ---               ---
 |   |-------------|   |
 | 1 |-------------| 2 |
  ---               ---
*/

#include "chassis_task.h"
#include "usart_debug.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//定义键鼠wsad普通速度及shift加速后速度
#define SPEED_LOW 3000
#define SPEED_HIGH 5400
#define SPEED_ROLE 3000
#define SPEED_FLY 13000
//普通模式遥控器转速度比例定义
#define NormalRCtoSpeedRatioX_Y 8
#define NormalRCtoSpeedRatioZ 8


//定义飞坡后两轮速度比例
#define FlySlopeRatio 1.1f
//飞坡模式遥控器转速度比例定义
#define FlyRCtoSpeedRatioX_Y 11
#define FlyRCtoSpeedRatioZ 6


#define Key_ctrl  1


#define JSCOPE_WATCH_chassis 0
#if JSCOPE_WATCH_chassis
//j-scope
static void Jscope_Watch_chassis(void);
#endif


Chassis_Control_t chassis_control;

static void Chassis_Init(Chassis_Control_t *chassis_init);
static void Chassis_Feedback_Update(Chassis_Control_t *chassis_feedback);
static void Chassis_Set_Control(Chassis_Control_t *chassis_set);
static void Chassis_PID(Chassis_Control_t *chassis_pid);
//static void chassis_power_control(Chassis_Control_t *chassis_set,uint16_t current_limit,uint16_t t);

static void Chassis_CMD(Chassis_Control_t *chassis_cmd);

const supercap_measure_t *cap;

uint8_t busV_low_mode;



void chassis_task(void const * argument)
{

	uint32_t waitetime;

	//底盘初始化
	Chassis_Init(&chassis_control);
	osDelay(1000);

	waitetime = xTaskGetTickCount();
	for (;;)
	{
		Chassis_Feedback_Update(&chassis_control);
		
		Chassis_Set_Control(&chassis_control);
		Chassis_PID(&chassis_control);
    
    Chassis_CMD(&chassis_control);

		osDelayUntil(&waitetime, CHASSIS_TASK_CONTROL_TIME);
	}
}



/*
  1.获取遥控器指针
  2.电机初始化
*/
void Chassis_Init(Chassis_Control_t *chassis_init)
{
	uint8_t i;
	if (chassis_init == NULL)
	{
		return ;
	}
	
	//获取遥控器指针
	chassis_init->chassis_RC = get_remote_control_point();
	//获取舵向指针
  chassis_init->chassis_swerve = get_swerve_motor_point();
	cap= get_SuperCap_Measure_Point();
	for (i=0 ; i<4 ; i++)
	{
		chassis_init->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
		PID_Init(&chassis_init->chassis_motor[i].speed_pid, CHASSIS_PID_MODE,CHASSIS_PID_MAX_OUT,CHASSIS_PID_MAX_IOUT,CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD);

	}
	//底盘裁判系统数据获取
  chassis_init->chassis_powermax = get_Chassis_PowerMax_Point();
	chassis_init->chassis_buffer = get_Chassis_Buffer_Point();
	//超级电容指针获取
  chassis_init->cap = get_SuperCap_Measure_Point();
}


/*
  获取电机反馈速度
*/
void Chassis_Feedback_Update(Chassis_Control_t *chassis_feedback)
{
	if (chassis_feedback == NULL)
    {
        return;
    }
	
	uint8_t i;
	for (i=0; i<4; i++)
	{
		chassis_feedback->chassis_motor[i].speed = chassis_feedback->chassis_motor[i].chassis_motor_measure->speed_rpm;	
	}
}


/*    
*/
void Chassis_Set_Control(Chassis_Control_t *chassis_set)
{
	if (chassis_set == NULL)
    {
        return;
    }
	
	
//	 chassis_set->chassis_motor[0].speed_set = -chassis_set->chassis_swerve->chassismotorspeed_algorithm[0];
//	 chassis_set->chassis_motor[1].speed_set = -chassis_set->chassis_swerve->chassismotorspeed_algorithm[1];
//	 chassis_set->chassis_motor[2].speed_set =  chassis_set->chassis_swerve->chassismotorspeed_algorithm[2];
//	 chassis_set->chassis_motor[3].speed_set =  chassis_set->chassis_swerve->chassismotorspeed_algorithm[3];

	
	 chassis_set->chassis_motor[0].speed_set = -chassis_set->chassis_swerve->chassismotorspeed_algorithm[0]*chassis_set->chassis_swerve->direction_change[0];
	 chassis_set->chassis_motor[1].speed_set = -chassis_set->chassis_swerve->chassismotorspeed_algorithm[1]*chassis_set->chassis_swerve->direction_change[1];
	 chassis_set->chassis_motor[2].speed_set =  chassis_set->chassis_swerve->chassismotorspeed_algorithm[2]*chassis_set->chassis_swerve->direction_change[2];
	 chassis_set->chassis_motor[3].speed_set =  chassis_set->chassis_swerve->chassismotorspeed_algorithm[3]*chassis_set->chassis_swerve->direction_change[3];
}

static void Chassis_PID(Chassis_Control_t *chassis_pid)
{
	uint8_t i;
	if (chassis_pid == NULL)
    {
        return;
    }
	
	for (i=0;i<4;i++)
	{
		
		if(chassis_pid->cap->capvolt/1000 < 13.5)
		{
				if(*chassis_pid->chassis_buffer>25)
				{
			    chassis_pid->chassis_motor[i].speed_set=0.8*chassis_pid->chassis_motor[i].speed_set;//该限幅是在地胶平地上测试得出的结果，后续需在起伏路段上测试后修正参数
				}
				else
        {
			    chassis_pid->chassis_motor[i].speed_set=0.6*chassis_pid->chassis_motor[i].speed_set;//该限幅是在地胶平地上测试得出的结果，后续需在起伏路段上测试后修正参数
				}
		}
			
		
		PID_Calculate(&chassis_pid->chassis_motor[i].speed_pid, 
			chassis_pid->chassis_motor[i].speed, chassis_pid->chassis_motor[i].speed_set);
		
		
			if(chassis_pid->cap->capvolt/1000 < 13.5)
			{
			  LimitMax(chassis_pid->chassis_motor[i].speed_pid.out,1800);//该限幅是在地胶平地上测试得出的结果，后续需在起伏路段上测试后修正参数
			}
	}
}

static void Chassis_CMD(Chassis_Control_t *chassis_cmd)
{
	    if(Swerve_start_flag == 0)
			{
				CAN_CMD_Chassis(0,0,0,0);
      }
	  	else
			{
				CAN_CMD_Chassis(chassis_cmd->chassis_motor[0].speed_pid.out
											 ,chassis_cmd->chassis_motor[1].speed_pid.out
											 ,chassis_cmd->chassis_motor[2].speed_pid.out
											 ,chassis_cmd->chassis_motor[3].speed_pid.out);
			}

}




//void chassis_power_control(Chassis_Control_t *chassis_set,uint16_t current_limit,uint16_t t)
//{
//	chassis_control.chassis_motor[t].speed_pid.max_out 	= 	current_limit;
//	chassis_control.chassis_motor[t].speed_pid.max_iout = 	current_limit/3;
//}

const Chassis_Control_t *get_motor_control_point(void)
{
    return &chassis_control;
}


#if JSCOPE_WATCH_chassis
float jlook_M1_speed;
float jlook_M2_speed;
float jlook_M3_speed;
float jlook_M4_speed;
float jlook_M1_setspeed;
float jlook_M2_setspeed;
float jlook_M3_setspeed;
float jlook_M4_setspeed;

float chassis_power;

float jlook_CH2;
float jlook_CH3;


static void Jscope_Watch_chassis(void)
{
	jlook_M1_speed = chassis_control.chassis_motor[0].speed;
	jlook_M2_speed = chassis_control.chassis_motor[1].speed;
	jlook_M3_speed = chassis_control.chassis_motor[2].speed;
	jlook_M4_speed = chassis_control.chassis_motor[3].speed;
	
	jlook_M1_setspeed=chassis_control.chassis_motor[0].speed_set;
	jlook_M2_setspeed=chassis_control.chassis_motor[1].speed_set;
	jlook_M3_setspeed=chassis_control.chassis_motor[2].speed_set;
	jlook_M4_setspeed=chassis_control.chassis_motor[3].speed_set;
	
	jlook_CH2 = chassis_control.chassis_RC->rc.ch[2];
	jlook_CH3 = chassis_control.chassis_RC->rc.ch[3];
	
	chassis_power = ina226_read(power);
}
#endif
