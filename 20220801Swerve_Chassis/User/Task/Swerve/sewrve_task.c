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

#include "swerve_task.h"
#include "usart_debug.h"
#include "chassis_task.h"
#include "math.h"
#include "filter.h"
#include "can_receive.h"
#define int_abs(x) ((x) > 0 ? (x) : (-x))

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
    
#define LimitMaxAngle(input, max1, max2)   \
    {                          \
        if (input > max1)       \
        {                      \
            input = max1;       \
        }                      \
        else if (input < -max2) \
        {                      \
            input = -max2;      \
        }                      \
    }


#define JSCOPE_WATCH_chassis 0
#if JSCOPE_WATCH_chassis
//j-scope
static void Jscope_Watch_chassis(void);
#endif

#define NormalRCtoSpeedRatioX_Y 8
#define NormalRCtoSpeedRatioZ 8
	
	
Swerve_Control_t swerve_control;

static void Swerve_Init(Swerve_Control_t *swerve_init);
static void Swerve_Feedback_Update(Swerve_Control_t *swerve_feedback);
static void Swerve_Set_Control(Swerve_Control_t *swerve_set);
static void Swerve_PID(Swerve_Control_t *swerve_pid);
static void Chassis_dirction_trainsfer(Swerve_Control_t *swerve_trainsfer);
static void Swerve_CMD(Swerve_Control_t *swerve_cmd);
static float Swerve_AnglePID_Calculate(PID_Regulator_t *pid , float fdb, float set);
static float Swerve_EcdPID_Calculate(PID_Regulator_t *pid , float fdb, float set);
static int16_t __abs(int16_t value);
float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd) ;
		
Filter_t Swerve_Motor_febfliter[4];
Filter_t Swerve_Motor_outfliter[4];	

volatile uint8_t Swerve_start_flag=0;

int16_t Chassis_Move_X=0;//????????
int16_t Chassis_Move_Y=0;//????????
int16_t Chassis_Move_Z=0;//????????
float swervetheta[4]={0};
uint16_t  stoptime_cnt=0;
uint32_t  runningtime_cnt=0;
uint16_t motor_speed_cnt;
//??????????????
int16_t Chassis_X=0;
int16_t Chassis_Y=0;


void swerve_task(void const * argument)
{
	uint32_t waitetime;
	Swerve_Init(&swerve_control);

	osDelay(2800);	


	waitetime = xTaskGetTickCount();
		while(1)
			{
			
				Swerve_Feedback_Update(&swerve_control);
				Swerve_Set_Control(&swerve_control);
				Swerve_PID(&swerve_control);
				Swerve_CMD(&swerve_control);		
				
			osDelayUntil(&waitetime,SWERVE_TASK_CONTROL_TIME);
			
			}
}

static void Swerve_Init(Swerve_Control_t *swerve_init)
{
		if (swerve_init == NULL)
    {
        return;
    }
	
  int i;
	//????????????????
	for(i=0;i<4;i++)
	{
	 swerve_init->swerve_motor[i].server_measure = get_Swerve_Gimbal_Motor_Measure_Point(i);
	}
	//YAW??????????????
	swerve_init->yaw_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	//??????????????????
  swerve_init->swerve_RC = get_remote_control_point();

	//????????PID??????
	for(i=0;i<4;i++)
	{
	PID_Init(&swerve_init->swerve_motor[i].speed_pid, SWERVE_SPEEDPID_MODE,SWERVE_SPEEDPID_MAX_OUT,SWERVE_SPEEDPID_MAX_IOUT,SWERVE_SPEEDPID_KP,SWERVE_SPEEDPID_KI,SWERVE_SPEEDPID_KD);
	PID_Init(&swerve_init->swerve_motor[i].position_pid, SWERVE_POSITIONPID_MODE,SWERVE_POSITIONPID_MAX_OUT,SWERVE_POSITIONPID_MAX_IOUT,SWERVE_POSITIONPID_KP,SWERVE_POSITIONPID_KI,SWERVE_POSITIONPID_KD);
	}
  //????????????PID??????
	PID_Init(&swerve_init->rotate_pid, CHASSIS_ROTATE_PID_MODE,CHASSIS_ROTATE_PID_MAX_OUT,CHASSIS_ROTATE_PID_MAX_IOUT,CHASSIS_ROTATE_PID_KP,CHASSIS_ROTATE_PID_KI, CHASSIS_ROTATE_PID_KD);


	//??????????????????
	swerve_init->swerve_motor[0].offset_ecd = SWREVE0_OFFSET_ECD;
	swerve_init->swerve_motor[1].offset_ecd = SWREVE1_OFFSET_ECD;
  swerve_init->swerve_motor[2].offset_ecd = SWREVE2_OFFSET_ECD;
  swerve_init->swerve_motor[3].offset_ecd = SWREVE3_OFFSET_ECD;

	swerve_init->angle45=atan2(1.0,1.0);
	swerve_init->angle90=atan2(1.0,0.0);
	swerve_init->angle180=atan2(0.0,-1.0);//??????????????????????3.14159274????????3.14159265
	swerve_init->angle270=atan2(0.0,-1.0)+atan2(1.0,0.0);
}

static void Swerve_Feedback_Update(Swerve_Control_t *swerve_feedback)
{
	if (swerve_feedback == NULL)
    {
        return;
    }
  int i;
			
	//????????????????
	swerve_feedback->dir.get_dir=get_Swerve_Direction_Point();
		
 	//????????????????????
	for(i=0;i<4;i++)
	{
	swerve_feedback->swerve_motor[i].ecd_fdb	= swerve_feedback->swerve_motor[i].server_measure->ecd;
	swerve_feedback->swerve_motor[i].relative_angle = Motor_ecd_to_angle_Change(swerve_feedback->swerve_motor[i].ecd_fdb,swerve_feedback->swerve_motor[i].offset_ecd);

	}
  //??????????????????	
	for(i=0;i<4;i++)
	{
	swerve_feedback->swerve_motor[i].gyro = swerve_feedback->swerve_motor[i].server_measure->speed_rpm;
	}
  //????????????????????
	swerve_feedback->yaw_ecderror=swerve_feedback->yaw_measure->ecd - YAW_OFFSET_ECD;
	swerve_feedback->chassis_yaw_relative_angle=Motor_ecd_to_angle_Change(swerve_feedback->yaw_measure->ecd,
																			YAW_OFFSET_ECD);
}
//static void Swerve_dirction_trainsfer(Swerve_Control_t *swerve_trainsfer)
//{


//}
static void Swerve_Set_Control(Swerve_Control_t *swerve_set)
{
	if (swerve_set == NULL)
    {
        return;
    }	

		if(((swerve_set->dir.get_dir->Swerve_Y!=0)||(swerve_set->dir.get_dir->Swerve_X!=0)||(swerve_set->dir.get_dir->Swerve_Z!=0)))
		{
			if(swerve_set->dir.get_dir->Swerve_Y>10000 || swerve_set->dir.get_dir->Swerve_X>10000 || swerve_set->dir.get_dir->Swerve_Z>10000)//??????????????????10000????????????????????
			{
				Swerve_start_flag=0;
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
			}
			else
			{
				Swerve_start_flag=1;
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
			}
		}
    else HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		
		
		uint8_t i=0;
//		Chassis_Move_X=0;//????????
//		Chassis_Move_Y=0;//????????
		Chassis_Move_Z=0;//????????
		
		
		motor_speed_cnt=0;
  
		Chassis_dirction_trainsfer(swerve_set);
	
//    if(Chassis_Move_X < swerve_set->dir.local_front)
//		  Chassis_Move_X += 8;
//    else if(Chassis_Move_X > swerve_set->dir.local_front)
//		  Chassis_Move_X -= 8;
//    if((Chassis_Move_X>=-4 && Chassis_Move_X<=4) || Swerve_start_flag==0)
//		  Chassis_Move_X = 0;
//	
//    if(Chassis_Move_Y < swerve_set->dir.local_left)
//		  Chassis_Move_Y += 8;
//    else if(Chassis_Move_Y > swerve_set->dir.local_left)
//		  Chassis_Move_Y -= 8;
//    if((Chassis_Move_Y>=-4 && Chassis_Move_Y<=4) || Swerve_start_flag==0)
//		  Chassis_Move_Y = 0;

			Chassis_Move_X = swerve_set->dir.local_front;
			Chassis_Move_Y = swerve_set->dir.local_left;

		if(ABS(swerve_set->dir.get_dir->Swerve_Z)>200)
		{
		  if(Swerve_start_flag==1) 
			{
				Chassis_Move_Z += swerve_set->dir.get_dir->Swerve_Z;
			}
			else Chassis_Move_Z=0;
		}
		else//????????
		{
			if(swerve_set->chassis_yaw_relative_angle>0.1 || swerve_set->chassis_yaw_relative_angle<-0.1)
			{
				Chassis_Move_Z += swerve_set->dir.get_dir->Swerve_Z - PID_Calculate(&swerve_set->rotate_pid,swerve_set->chassis_yaw_relative_angle,0);
			}
			else 
			{
				Chassis_Move_Z += swerve_set->dir.get_dir->Swerve_Z;			
			}
		}




		swervetheta[0]=swerve_set->angle45;
		swervetheta[1]=swerve_set->angle180-swerve_set->angle45;//135
		swervetheta[2]=swerve_set->angle45-swerve_set->angle180;//-135
		swervetheta[3]=-swerve_set->angle45;//-45
	
		for(i=0;i<4;i++)//????3508????????
		{
			swerve_set->chassismotorspeed_algorithm	[i]//????3508????????
			= sqrt(pow(Chassis_Move_X + Chassis_Move_Z*arm_sin_f32(swervetheta[i]),2) + pow(Chassis_Move_Y + Chassis_Move_Z*arm_cos_f32(swervetheta[i]),2));
			
			motor_speed_cnt+=__abs(swerve_set->chassismotorspeed_algorithm[i]);
		}

		if(motor_speed_cnt==0)//??????????????????(????100??????????????????)??????????????????,??????????????????????????????
		{
//			stoptime_cnt++;

				runningtime_cnt=0;
				swerve_set->swerve_motor[0].relative_angle_algorithm	 
				= swerve_set->swerve_motor[0].relative_angle_algorithm;
				swerve_set->swerve_motor[1].relative_angle_algorithm
				= swerve_set->swerve_motor[1].relative_angle_algorithm;
				swerve_set->swerve_motor[2].relative_angle_algorithm
				= swerve_set->swerve_motor[2].relative_angle_algorithm;
				swerve_set->swerve_motor[3].relative_angle_algorithm
				= swerve_set->swerve_motor[3].relative_angle_algorithm;
//				swerve_set->swerve_motor[0].relative_angle_algorithm	 
//				= atan2f( 1, 1);
//				swerve_set->swerve_motor[1].relative_angle_algorithm
//				= atan2f(-1, 1);
//				swerve_set->swerve_motor[2].relative_angle_algorithm
//				= atan2f(-1,-1);
//				swerve_set->swerve_motor[3].relative_angle_algorithm
//			 = atan2f( 1,-1);
		  
		}
    else
    {

			runningtime_cnt++;
//			stoptime_cnt=0;
			if(runningtime_cnt>=5)
			{
				runningtime_cnt=5;
				swerve_set->swerve_motor[0].relative_angle_algorithm	 //????????????????,??????atan2??????atan
				= atan2f(Chassis_Move_Y + Chassis_Move_Z*arm_cos_f32(swerve_set->angle45),Chassis_Move_X + Chassis_Move_Z*arm_sin_f32(swerve_set->angle45));
				swerve_set->swerve_motor[1].relative_angle_algorithm
				= atan2f(Chassis_Move_Y - Chassis_Move_Z*arm_cos_f32(swerve_set->angle45),Chassis_Move_X + Chassis_Move_Z*arm_sin_f32(swerve_set->angle45));
				swerve_set->swerve_motor[2].relative_angle_algorithm
				= atan2f(Chassis_Move_Y - Chassis_Move_Z*arm_cos_f32(swerve_set->angle45),Chassis_Move_X - Chassis_Move_Z*arm_sin_f32(swerve_set->angle45));
				swerve_set->swerve_motor[3].relative_angle_algorithm
				= atan2f(Chassis_Move_Y + Chassis_Move_Z*arm_cos_f32(swerve_set->angle45),Chassis_Move_X- Chassis_Move_Z*arm_sin_f32(swerve_set->angle45));
			}
			

		}

		
	float angle_err=0;

	for(i=0;i<4;i++)//??????????????????????????
	{
//    swerve_set->swerve_motor[i].last_relative_angle_set = swerve_set->swerve_motor[i].relative_angle_set;//??????????????????
		angle_err = swerve_set->swerve_motor[i].relative_angle - swerve_set->swerve_motor[i].relative_angle_algorithm;
//			if (angle_err > swerve_set->angle180*2) swerve_set->swerve_motor[i].relative_angle_set -=2*swerve_set->angle180;
//		if (angle_err < 0) swerve_set->swerve_motor[i].relative_angle_set +=2*swerve_set->angle180;	
		if((__abs(angle_err) >= swerve_set->angle90) && (__abs(angle_err) < swerve_set->angle270))//????????????????????????90??????????270????
		{
		swerve_set->swerve_motor[i].relative_angle_set = swerve_set->swerve_motor[i].relative_angle_algorithm - swerve_set->angle180;
		swerve_set->direction_change[i]=-1;
		}
		else
		{
	  swerve_set->swerve_motor[i].relative_angle_set = swerve_set->swerve_motor[i].relative_angle_algorithm;
		swerve_set->direction_change[i]=1;
		}
		
		if (swerve_set->swerve_motor[i].relative_angle_set > swerve_set->angle180)  swerve_set->swerve_motor[i].relative_angle_set -=2*swerve_set->angle180;
		if (swerve_set->swerve_motor[i].relative_angle_set <=-swerve_set->angle180) swerve_set->swerve_motor[i].relative_angle_set +=2*swerve_set->angle180;	
		
//		if(__abs(swerve_set->swerve_motor[i].relative_angle_set-swerve_set->swerve_motor[i].relative_angle_algorithm) < 0.001)
//		{
//		swerve_set->direction_change[i]=1;
//		}
//		else swerve_set->direction_change[i]=-1;

		
//			if((Chassis_Move_Y!=0)||(Chassis_Move_X!=0))//????????????????????????????????
//			{

//			  	  swerve_set->swerve_motor[i].relative_angle_set -= swerve_set->chassis_yaw_relative_angle;


//			      if(swerve_set->swerve_motor[i].relative_angle_set > swerve_set->angle180)
//			        swerve_set->swerve_motor[i].relative_angle_set -= 2*swerve_set->angle180;
//            else if(swerve_set->swerve_motor[i].relative_angle_set <= -swerve_set->angle180)
//			        swerve_set->swerve_motor[i].relative_angle_set += 2*swerve_set->angle180;

//			}
		
		  //????????ecdset??,????ecd????????
//			if(swerve_set->swerve_motor[i].relative_angle_algorithm!=0)
//			{
		swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].relative_angle_set/swerve_set->angle180*HALFECD + swerve_set->swerve_motor[i].offset_ecd;

//		swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].relative_angle_algorithm/swerve_set->angle180*HALFECD + swerve_set->swerve_motor[i].offset_ecd;
	
		//			}
//			else if(swerve_set->swerve_motor[i].relative_angle_algorithm==0)
//			{
//			swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].offset_ecd;
//			}

		
		
		//??????????ecd??????
		swerve_set->swerve_motor[i].ecd_algorithm=(int16_t)(swerve_set->swerve_motor[i].ecd_algorithm+0.5);//0.5??????????????????
		if(swerve_set->swerve_motor[i].ecd_algorithm < 0)
			swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].ecd_algorithm  + FULLECD;
    else if(swerve_set->swerve_motor[i].ecd_algorithm >= FULLECD)
			swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].ecd_algorithm  - FULLECD;


//	  if((swerve_set->swerve_motor[i].ecd_algorithm - swerve_set->swerve_motor[i].ecd_fdb) > QUATERECD)//??????????????????????????????????90????
//	  {
//	   swerve_set->swerve_motor[i].ecd_set= swerve_set->swerve_motor[i].ecd_algorithm - HALFECD;//????????????
//	  }
//		else if((swerve_set->swerve_motor[i].ecd_algorithm - swerve_set->swerve_motor[i].ecd_fdb) < -QUATERECD)
//	  {
//	   swerve_set->swerve_motor[i].ecd_set= swerve_set->swerve_motor[i].ecd_algorithm + HALFECD;
//	  }
//	  else
//    {
//     swerve_set->swerve_motor[i].ecd_set=swerve_set->swerve_motor[i].ecd_algorithm;
//    }
    //??????????
//		if(swerve_set->swerve_motor[i].ecd_set < 0) 
//		swerve_set->swerve_motor[i].ecd_set=swerve_set->swerve_motor[i].ecd_set + 8192;//0.5??????????????????
//    else if(swerve_set->swerve_motor[i].ecd_set >= 8192)
//		swerve_set->swerve_motor[i].ecd_set=swerve_set->swerve_motor[i].ecd_set  - 8192;
//		//????????????
//	  if(swerve_set->swerve_motor[i].ecd_set!=swerve_set->swerve_motor[i].ecd_algorithm)
//	  {
//	  swerve_set->chassismotor[i].speed_set=-swerve_set->chassismotor[i].speed_algorithm;
//		}
//		else if(swerve_set->swerve_motor[i].ecd_set==swerve_set->swerve_motor[i].ecd_algorithm)
//		{
//		chassis_control.chassis_motor[i].speed_set=chassis_control.chassis_motor[i].speed_algorithm;
//		}
		
	
 }

//   	for(i=0;i<4;i++)//????????????????
//  	{
//	   if(swerve_set->swerve_motor[i].relative_angle_set-swerve_set->swerve_motor[i].relative_angle <= 0.0007f && 
//      swerve_set->swerve_motor[i].relative_angle_set-swerve_set->swerve_motor[i].relative_angle>= -0.0007f)
//      {
//        swerve_set->swerve_motor[i].relative_angle_set = swerve_set->swerve_motor[i].relative_angle;
//		  }


}

static void Swerve_PID(Swerve_Control_t *swerve_pid)
{
	
	

	int i;
	for(i=0;i<4;i++)//????????????pid????
  	{

			Swerve_EcdPID_Calculate(&swerve_pid->swerve_motor[i].position_pid, swerve_pid->swerve_motor[i].ecd_fdb,(int16_t)swerve_pid->swerve_motor[i].ecd_algorithm);

			PID_Calculate(&swerve_pid->swerve_motor[i].speed_pid,
										(float)swerve_pid->swerve_motor[i].server_measure->speed_rpm,
										(float)swerve_pid->swerve_motor[i].position_pid.out);

		}
}

static void Swerve_CMD(Swerve_Control_t *swerve_cmd)
{
//	int i=0;
//	for(i=0;i<4;i++)
//	{
//		Swerve_Motor_outfliter[i].raw_value = swerve_cmd->swerve_motor[i].position_pid.out;	
//		Chebyshev100HzLPF(&Swerve_Motor_outfliter[i]);
//	}
	

	
	    if(Swerve_start_flag == 0)
			{
		   CAN_CMD_Swerve(0,0,0,0);
      }
			else
			{
//				 CAN_CMD_Swerve(Swerve_Motor_outfliter[0].filtered_value,Swerve_Motor_outfliter[1].filtered_value,Swerve_Motor_outfliter[2].filtered_value,Swerve_Motor_outfliter[3].filtered_value);
				
	     CAN_CMD_Swerve(swerve_cmd->swerve_motor[0].speed_pid.out,swerve_cmd->swerve_motor[1].speed_pid.out,swerve_cmd->swerve_motor[2].speed_pid.out,swerve_cmd->swerve_motor[3].speed_pid.out);
			}


}

static void Chassis_dirction_trainsfer(Swerve_Control_t *swerve_trainsfer)
{
	//
	swerve_trainsfer->dir.M_global_local[0] =  arm_cos_f32(swerve_trainsfer->chassis_yaw_relative_angle);
	swerve_trainsfer->dir.M_global_local[1] =  arm_sin_f32(swerve_trainsfer->chassis_yaw_relative_angle);
	swerve_trainsfer->dir.M_global_local[2] = -arm_sin_f32(swerve_trainsfer->chassis_yaw_relative_angle);
	swerve_trainsfer->dir.M_global_local[3] =  arm_cos_f32(swerve_trainsfer->chassis_yaw_relative_angle);
	
	//??????????
	if(Chassis_X < swerve_trainsfer->dir.get_dir->Swerve_X)
	{
	  Chassis_X+=8;
	}
	else if(Chassis_X > swerve_trainsfer->dir.get_dir->Swerve_X)
	{
		Chassis_X-=8;	
	}
	if((Chassis_X<=4 && Chassis_X>=-4) || Swerve_start_flag == 0)
	{
	  Chassis_X=0;
	}
	
	if(Chassis_Y < swerve_trainsfer->dir.get_dir->Swerve_Y)
	{
		Chassis_Y+=8;
	}
	else if(Chassis_Y > swerve_trainsfer->dir.get_dir->Swerve_Y)
	{
		Chassis_Y-=8;	
	}
	if((Chassis_Y<=4 && Chassis_Y>=-4) || Swerve_start_flag == 0)
	{
	  Chassis_Y=0;	
	}
	
	
	swerve_trainsfer->dir.local_front = 
		-(Chassis_Y * arm_sin_f32(swerve_trainsfer->chassis_yaw_relative_angle) +
		  Chassis_X * arm_cos_f32(swerve_trainsfer->chassis_yaw_relative_angle));
	
	swerve_trainsfer->dir.local_left =
		-(Chassis_Y * arm_cos_f32(swerve_trainsfer->chassis_yaw_relative_angle) +
		  Chassis_X * -arm_sin_f32(swerve_trainsfer->chassis_yaw_relative_angle));
	
	if(__abs(swerve_trainsfer->dir.local_front)<20) swerve_trainsfer->dir.local_front=0;
	if(__abs(swerve_trainsfer->dir.local_left)<20) swerve_trainsfer->dir.local_left=0;	
}


static float Swerve_AnglePID_Calculate(PID_Regulator_t *pid , float fdb, float set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }
	
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];

	pid->set = set + PI;
	pid->fdb = fdb + PI;
	//????????????????
	if (pid->set - pid->fdb > PI)
		pid->err[0] = pid->set - pid->fdb -2*PI;
	else if  (pid->set - pid->fdb < -PI)
		pid->err[0] = pid->set - pid->fdb +2*PI;
	else pid->err[0] = pid->set - pid->fdb;
	
	if (pid->mode == PID_POSITION)
	{
		pid->Pout = pid->kp * pid->err[0];
		pid->Iout += pid->ki * pid->err[0];
		pid->Dout = pid->kd *(pid->err[0] - pid->err[1]);
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	else if (pid->mode == PID_DELTA)
	{
		pid->Pout = pid->kp * (pid->err[0] - pid->err[1]);
		pid->Iout = pid->ki * pid->err[0];
		 pid->Dout = pid->kd * (pid->err[0] - 2.0f * pid->err[1] + pid->err[2]);
		pid->out += pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	return pid->out;
	
}

static float Swerve_EcdPID_Calculate(PID_Regulator_t *pid , float fdb, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];

	  pid->set = set;
    pid->fdb = fdb;

		if (pid->set - pid->fdb > 4096)
			pid->err[0] = pid->set - pid->fdb -2*4096;
		else if  (pid->set - pid->fdb < -4096)
			pid->err[0] = pid->set - pid->fdb +2*4096;
		else pid->err[0] = pid->set - pid->fdb;
    
	  if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->kp * pid->err[0];
        pid->Iout += pid->ki * pid->err[0];
        pid->Dout = pid->kd *(pid->err[0] - pid->err[1]);
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->kp * (pid->err[0] - pid->err[1]);
        pid->Iout = pid->ki * pid->err[0];
        pid->Dout = pid->kd * (pid->err[0] - 2.0f * pid->err[1] + pid->err[2]);
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

const Swerve_Control_t *get_swerve_motor_point(void)
{
	return &swerve_control;
}


static int16_t __abs(int16_t value)
{
    if (value >= 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}


//????????????????????????????????????
float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd) 
{
	//??????????????????????
	int32_t relative_ecd = ecd - offset_ecd;
	
	//??????????4096??????????????8192??????????????????????????
    if (relative_ecd > HALF_RANGE)
    {
        relative_ecd -= FULL_RANGE;
    }
	//??????????-4096??????????????8192??????????????????????????
    else if (relative_ecd < -HALF_RANGE)
    {
        relative_ecd += FULL_RANGE;
    }
	
    return relative_ecd * Motor_Ecd_to_Rad;
}
