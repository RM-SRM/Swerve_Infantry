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

int16_t Chassis_Move_X=0;//前后方向
int16_t Chassis_Move_Y=0;//左右方向
int16_t Chassis_Move_Z=0;//旋转方向
float swervetheta[4]={0};
uint16_t  stoptime_cnt=0;
uint32_t  runningtime_cnt=0;
uint16_t motor_speed_cnt;
//用于做速度斜坡
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
	//电机数据指针获取
	for(i=0;i<4;i++)
	{
	 swerve_init->swerve_motor[i].server_measure = get_Swerve_Gimbal_Motor_Measure_Point(i);
	}
	//YAW轴电机数据获取
	swerve_init->yaw_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	//遥控器数据指针获取
  swerve_init->swerve_RC = get_remote_control_point();

	//舵轮电机PID初始化
	for(i=0;i<4;i++)
	{
	PID_Init(&swerve_init->swerve_motor[i].speed_pid, SWERVE_SPEEDPID_MODE,SWERVE_SPEEDPID_MAX_OUT,SWERVE_SPEEDPID_MAX_IOUT,SWERVE_SPEEDPID_KP,SWERVE_SPEEDPID_KI,SWERVE_SPEEDPID_KD);
	PID_Init(&swerve_init->swerve_motor[i].position_pid, SWERVE_POSITIONPID_MODE,SWERVE_POSITIONPID_MAX_OUT,SWERVE_POSITIONPID_MAX_IOUT,SWERVE_POSITIONPID_KP,SWERVE_POSITIONPID_KI,SWERVE_POSITIONPID_KD);
	}
  //舵轮底盘跟随PID初始化
	PID_Init(&swerve_init->rotate_pid, CHASSIS_ROTATE_PID_MODE,CHASSIS_ROTATE_PID_MAX_OUT,CHASSIS_ROTATE_PID_MAX_IOUT,CHASSIS_ROTATE_PID_KP,CHASSIS_ROTATE_PID_KI, CHASSIS_ROTATE_PID_KD);


	//舵轮电机中值初始化
	swerve_init->swerve_motor[0].offset_ecd = SWREVE0_OFFSET_ECD;
	swerve_init->swerve_motor[1].offset_ecd = SWREVE1_OFFSET_ECD;
  swerve_init->swerve_motor[2].offset_ecd = SWREVE2_OFFSET_ECD;
  swerve_init->swerve_motor[3].offset_ecd = SWREVE3_OFFSET_ECD;

	swerve_init->angle45=atan2(1.0,1.0);
	swerve_init->angle90=atan2(1.0,0.0);
	swerve_init->angle180=atan2(0.0,-1.0);//根据公式结算出的Π值为3.14159274，而不是3.14159265
	swerve_init->angle270=atan2(0.0,-1.0)+atan2(1.0,0.0);
}

static void Swerve_Feedback_Update(Swerve_Control_t *swerve_feedback)
{
	if (swerve_feedback == NULL)
    {
        return;
    }
  int i;
			
	//获取底盘运动数据
	swerve_feedback->dir.get_dir=get_Swerve_Direction_Point();
		
 	//电机相对角度数据更新
	for(i=0;i<4;i++)
	{
	swerve_feedback->swerve_motor[i].ecd_fdb	= swerve_feedback->swerve_motor[i].server_measure->ecd;
	swerve_feedback->swerve_motor[i].relative_angle = Motor_ecd_to_angle_Change(swerve_feedback->swerve_motor[i].ecd_fdb,swerve_feedback->swerve_motor[i].offset_ecd);

	}
  //电机角速度数据更新	
	for(i=0;i<4;i++)
	{
	swerve_feedback->swerve_motor[i].gyro = swerve_feedback->swerve_motor[i].server_measure->speed_rpm;
	}
  //底盘跟随相对角度计算
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
			if(swerve_set->dir.get_dir->Swerve_Y>10000 || swerve_set->dir.get_dir->Swerve_X>10000 || swerve_set->dir.get_dir->Swerve_Z>10000)//当云台传来的值大于10000时，底盘进入断电模式
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
//		Chassis_Move_X=0;//前后方向
//		Chassis_Move_Y=0;//左右方向
		Chassis_Move_Z=0;//旋转方向
		
		
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
		else//底盘跟随
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
	
		for(i=0;i<4;i++)//底盘3508速度计算
		{
			swerve_set->chassismotorspeed_algorithm	[i]//底盘3508速度计算
			= sqrt(pow(Chassis_Move_X + Chassis_Move_Z*arm_sin_f32(swervetheta[i]),2) + pow(Chassis_Move_Y + Chassis_Move_Z*arm_cos_f32(swervetheta[i]),2));
			
			motor_speed_cnt+=__abs(swerve_set->chassismotorspeed_algorithm[i]);
		}

		if(motor_speed_cnt==0)//当轮子转速设定为零(设定100是为了消除微小误差)，且持续一段时间时,使舵向电机转向为小陀螺时的角度
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
				swerve_set->swerve_motor[0].relative_angle_algorithm	 //舵轮电机角度结算,注意是atan2而不是atan
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

	for(i=0;i<4;i++)//对解算的角度和速度进行矫正
	{
//    swerve_set->swerve_motor[i].last_relative_angle_set = swerve_set->swerve_motor[i].relative_angle_set;//保存上次的角度设定
		angle_err = swerve_set->swerve_motor[i].relative_angle - swerve_set->swerve_motor[i].relative_angle_algorithm;
//			if (angle_err > swerve_set->angle180*2) swerve_set->swerve_motor[i].relative_angle_set -=2*swerve_set->angle180;
//		if (angle_err < 0) swerve_set->swerve_motor[i].relative_angle_set +=2*swerve_set->angle180;	
		if((__abs(angle_err) >= swerve_set->angle90) && (__abs(angle_err) < swerve_set->angle270))//当当前角度与解算角度大于90度时且小于270度时
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

		
//			if((Chassis_Move_Y!=0)||(Chassis_Move_X!=0))//对解算的角度加上底盘跟随相对角度
//			{

//			  	  swerve_set->swerve_motor[i].relative_angle_set -= swerve_set->chassis_yaw_relative_angle;


//			      if(swerve_set->swerve_motor[i].relative_angle_set > swerve_set->angle180)
//			        swerve_set->swerve_motor[i].relative_angle_set -= 2*swerve_set->angle180;
//            else if(swerve_set->swerve_motor[i].relative_angle_set <= -swerve_set->angle180)
//			        swerve_set->swerve_motor[i].relative_angle_set += 2*swerve_set->angle180;

//			}
		
		  //反解算出ecdset值,使用ecd进行闭环
//			if(swerve_set->swerve_motor[i].relative_angle_algorithm!=0)
//			{
		swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].relative_angle_set/swerve_set->angle180*HALFECD + swerve_set->swerve_motor[i].offset_ecd;

//		swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].relative_angle_algorithm/swerve_set->angle180*HALFECD + swerve_set->swerve_motor[i].offset_ecd;
	
		//			}
//			else if(swerve_set->swerve_motor[i].relative_angle_algorithm==0)
//			{
//			swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].offset_ecd;
//			}

		
		
		//将解算出的ecd值归正
		swerve_set->swerve_motor[i].ecd_algorithm=(int16_t)(swerve_set->swerve_motor[i].ecd_algorithm+0.5);//0.5是为了四舍五入取整
		if(swerve_set->swerve_motor[i].ecd_algorithm < 0)
			swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].ecd_algorithm  + FULLECD;
    else if(swerve_set->swerve_motor[i].ecd_algorithm >= FULLECD)
			swerve_set->swerve_motor[i].ecd_algorithm=swerve_set->swerve_motor[i].ecd_algorithm  - FULLECD;


//	  if((swerve_set->swerve_motor[i].ecd_algorithm - swerve_set->swerve_motor[i].ecd_fdb) > QUATERECD)//当电机解算角度与当前角度值之差大于90°时
//	  {
//	   swerve_set->swerve_motor[i].ecd_set= swerve_set->swerve_motor[i].ecd_algorithm - HALFECD;//电机角度反转
//	  }
//		else if((swerve_set->swerve_motor[i].ecd_algorithm - swerve_set->swerve_motor[i].ecd_fdb) < -QUATERECD)
//	  {
//	   swerve_set->swerve_motor[i].ecd_set= swerve_set->swerve_motor[i].ecd_algorithm + HALFECD;
//	  }
//	  else
//    {
//     swerve_set->swerve_motor[i].ecd_set=swerve_set->swerve_motor[i].ecd_algorithm;
//    }
    //设定值归正
//		if(swerve_set->swerve_motor[i].ecd_set < 0) 
//		swerve_set->swerve_motor[i].ecd_set=swerve_set->swerve_motor[i].ecd_set + 8192;//0.5是为了四舍五入取整
//    else if(swerve_set->swerve_motor[i].ecd_set >= 8192)
//		swerve_set->swerve_motor[i].ecd_set=swerve_set->swerve_motor[i].ecd_set  - 8192;
//		//速度解算调整
//	  if(swerve_set->swerve_motor[i].ecd_set!=swerve_set->swerve_motor[i].ecd_algorithm)
//	  {
//	  swerve_set->chassismotor[i].speed_set=-swerve_set->chassismotor[i].speed_algorithm;
//		}
//		else if(swerve_set->swerve_motor[i].ecd_set==swerve_set->swerve_motor[i].ecd_algorithm)
//		{
//		chassis_control.chassis_motor[i].speed_set=chassis_control.chassis_motor[i].speed_algorithm;
//		}
		
	
 }

//   	for(i=0;i<4;i++)//微小范围死区处理
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
	for(i=0;i<4;i++)//滤波完再进行pid计算
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
	
	//做速度斜坡
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
	//角度解算必要步骤
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


//将机器人水平正前方作为云台电机的零点
float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd) 
{
	//计算当前位置与零点的差
	int32_t relative_ecd = ecd - offset_ecd;
	
	//若差值大于4096，则将结果减去8192，说明电机此时在零点负一侧
    if (relative_ecd > HALF_RANGE)
    {
        relative_ecd -= FULL_RANGE;
    }
	//若差值小于-4096，则将结果加上8192，说明电机此时在零点正一侧
    else if (relative_ecd < -HALF_RANGE)
    {
        relative_ecd += FULL_RANGE;
    }
	
    return relative_ecd * Motor_Ecd_to_Rad;
}
