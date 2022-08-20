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
#include "PressTime.h"

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
#define SPEED_LOW 3300
#define SPEED_HIGH 5200
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


static Chassis_Control_t chassis_control;

static void Chassis_Init(Chassis_Control_t *chassis_init);
//static void Chassis_Feedback_Update(Chassis_Control_t *chassis_feedback);
static void Chassis_Set_Control(Chassis_Control_t *chassis_set);
//static void Chassis_PID(Chassis_Control_t *chassis_pid);
//static void chassis_power_control(Chassis_Control_t *chassis_set,uint16_t current_limit,uint16_t t);
static void Chassis_dirction_trainsfer(Chassis_Control_t *chassis_trainsfer);
static void Chassis_sendmessage(Chassis_Control_t *chassis_cmd);
const SuperCap_Control_t *SUPER_CAP;
const supercap_measure_t *cap;

uint8_t busV_low_mode=0;

extern volatile uint8_t imu_start_dma_flag;

void chassis_Task(void const * argument)
{
//	in226Init(&hi2c2);
	uint32_t waitetime;
    int i;

  
	//底盘初始化
	Chassis_Init(&chassis_control);
	osDelay(100);
	while(imu_start_dma_flag != 1)//|| mpu6500.temperature < TEMP_SET-0.2f)
    {
//    Chassis_Feedback_Update(&chassis_control);
		osDelay(1);
    }
	osDelay(300);

    
 
	
//	waitetime = xTaskGetTickCount();
	for (;;)
	{
//		Chassis_Feedback_Update(&chassis_control);
		
		Chassis_Set_Control(&chassis_control);
//		Chassis_PID(&chassis_control);
    
//    Chassis_sendmessage(&chassis_control);
		
			



	    
#if JSCOPE_WATCH_chassis		
		Jscope_Watch_chassis();
#endif
		
		osDelay(1);
//		osDelayUntil(&waitetime, GIMBAL_TASK_CONTROL_TIME);
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
//	//获取云台电机指针
//	chassis_init->chassis_yaw_motor = get_yaw_motor_point();	
//	chassis_init->chassis_pitch_motor = get_pitch_motor_point();
	//电机PID初始化
//	SUPER_CAP = get_Supercap_Control_Measure_Point();
	cap= get_SuperCap_Measure_Point();
//	for (i=0 ; i<4 ; i++)
//	{
//		chassis_init->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
//		PID_Init(&chassis_init->chassis_motor[i].speed_pid, CHASSIS_PID_MODE,CHASSIS_PID_MAX_OUT,CHASSIS_PID_MAX_IOUT,CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD);
////	  chassis_power_control(chassis_init,1500,i);
//	}
	
//	//底盘跟随PID初始化
//	PID_Init(&chassis_init->rotate_pid,CHASSIS_ROTATE_PID_MODE,CHASSIS_ROTATE_PID_MAX_OUT,CHASSIS_ROTATE_PID_MAX_IOUT,CHASSIS_ROTATE_PID_KP,CHASSIS_ROTATE_PID_KI,CHASSIS_ROTATE_PID_KD);
}


/*
  获取电机反馈速度
*/
//void Chassis_Feedback_Update(Chassis_Control_t *chassis_feedback)
//{
//	if (chassis_feedback == NULL)
//    {
//        return;
//    }
//	
//	uint8_t i;
//	for (i=0; i<4; i++)
//	{
//		chassis_feedback->chassis_motor[i].speed = chassis_feedback->chassis_motor[i].chassis_motor_measure->speed_rpm;	
//	}
//}


/*    
*/
void Chassis_Set_Control(Chassis_Control_t *chassis_set)
{
	uint8_t i=0;
	if (chassis_set == NULL)
    {
        return;
    }
	
	int16_t Chassis_Move_X=0;//前后方向
	int16_t Chassis_Move_Y=0;//左右方向
	int16_t Chassis_Move_Z=0;//左右旋转
		

#ifdef Key_ctrl
		 if(Q_mode==1&&E_mode==0){
			    Ctrl_mode=0;
	        if(Shift_mode==1)
					{
							Chassis_Move_Z +=-SPEED_HIGH;
					}
					else
					{
							Chassis_Move_Z +=-SPEED_ROLE;
					}
				}
	
	else if(Q_mode==0&&E_mode==1){
			
          Ctrl_mode=0;
					if(Shift_mode==1)
					{
							Chassis_Move_Z += SPEED_HIGH;
					}
					else
					{
							Chassis_Move_Z += SPEED_ROLE;
					}
				
			  }
	
else if(E_mode==1&&Q_mode==1)				
{
	      Ctrl_mode=1;
				Chassis_Move_Z += 0;
	      E_mode=0;
	      Q_mode=0;
				Q_cnt=0;
	      E_cnt=0;
}
else if(E_mode==0&&Q_mode==0)				
{
	
if(rc_ctrl.rc.sright==RC_MID)
	      Ctrl_mode=1;
else
	      Ctrl_mode=0;	
				Chassis_Move_Z += 0;
	      E_mode=0;
	      Q_mode=0;
}
		
		
	
	if(IF_KEY_PRESSED)
	{

			if(IF_KEY_PRESSED_W)
			{
					if(Shift_mode==1)
					{
							Chassis_Move_X = SPEED_HIGH;
					}
					else
					{
							Chassis_Move_X = SPEED_LOW;
  				}

      if(X_mode==1)
       {
			 Chassis_Move_X=13000;
			 shift_step=2;
			 }
			
				
			}
			
			if(IF_KEY_PRESSED_S)
			{
					if(Shift_mode==1)
					{
							Chassis_Move_X = -SPEED_HIGH;
					}
					else
					{
							Chassis_Move_X = -SPEED_LOW;
					}
			}

			if(IF_KEY_PRESSED_A)
			{
					if(Shift_mode==1)
					{
							Chassis_Move_Y = -SPEED_HIGH*2/3;
					}
					else
					{
							Chassis_Move_Y = -SPEED_LOW*2/3;
					}
			}
			
			if(IF_KEY_PRESSED_D)
			{
					if(Shift_mode==1)
					{
							Chassis_Move_Y = SPEED_HIGH*2/3;
					}
					else
					{
							Chassis_Move_Y = SPEED_LOW*2/3;
					}
			}
			if(IF_KEY_PRESSED_V||IF_KEY_PRESSED_B||IF_KEY_PRESSED_R||IF_KEY_PRESSED_G||IF_KEY_PRESSED_F
				||IF_KEY_PRESSED_Z||IF_KEY_PRESSED_X){
					//防止程序跑飞，检测到与底盘无关的其他按键时底盘不做任何响应
				
			}
		}
	

#endif				

	Chassis_Move_X += chassis_set->chassis_RC->rc.ch[3]*NormalRCtoSpeedRatioX_Y;
	Chassis_Move_Y += chassis_set->chassis_RC->rc.ch[2]*NormalRCtoSpeedRatioX_Y;
	Chassis_Move_Z += chassis_set->chassis_RC->rc.wheel*NormalRCtoSpeedRatioZ;  
		
	chassis_set->dir.global_front=Chassis_Move_X;
	chassis_set->dir.global_left=Chassis_Move_Y;
	chassis_set->dir.rotate=Chassis_Move_Z;
	
	if(cap->capvolt/1000.f < 13.5)
	{
		busV_low_mode=1;
	}
	else if(cap->capvolt/1000.f > 17.0)
	{
		busV_low_mode=0;	
	}		
	
	if(busV_low_mode==1)
	{
		 LimitMax(chassis_set->dir.global_front,2500);
		 LimitMax(chassis_set->dir.global_left,2000);
		 LimitMax(chassis_set->dir.rotate,2500);			
	}
		
	if((chassis_set->chassis_RC->rc.sleft==RC_DOWN&&chassis_set->chassis_RC->rc.sright==RC_DOWN)||chassis_set->chassis_RC->rc.sright==0)
	  CAN_CMD_Chassis(supercap_control.chassis_power_max ,supercap_control.chassis_power_buffer_read/2,15000,15000,15000);	
	else
	  CAN_CMD_Chassis(supercap_control.chassis_power_max ,supercap_control.chassis_power_buffer_read/2,chassis_set->dir.global_front,chassis_set->dir.global_left,chassis_set->dir.rotate);
//	Chassis_Move_X = 1000;
//	Chassis_Move_Y = 1000;
//	Chassis_Move_Z =  1000;
//if(cap->capvolt==0){
//	for (i=0;i<4;i++)
//	{
//		
//		if(SUPER_CAP->chassis_power_buffer_read<=27.5)
//			{
//			chassis_control.chassis_motor[i].speed_set=0.4*chassis_control.chassis_motor[i].speed_set;
//			}
//		}
//}
//else{
//			 if(((cap->capvolt/800.0f)<=15.2&&cap->mode==1)||(cap->capvolt/800.0f)<=13.0){
//						
//			Chassis_Move_X=Chassis_Move_X/3.2;
//      Chassis_Move_Y=Chassis_Move_Y/3.2;
//			Chassis_Move_Z=Chassis_Move_Z/3.2;
//						Shift_mode=0;
//						Shift_cnt=0;
//						shift_step=1;
//			}
//		}

//			
//		

//		if(chassis_set->dir.global_front<Chassis_Move_X)
//		{
//		chassis_set->dir.global_front+=10*shift_step;
//		}
//		else if(chassis_set->dir.global_front>Chassis_Move_X)
//		{
//		chassis_set->dir.global_front-=16*shift_step;
//		}
//		else{
//			chassis_set->dir.global_front = Chassis_Move_X;
//		}
//		
//		if(chassis_set->dir.global_left<Chassis_Move_Y){
//		chassis_set->dir.global_left+=10*shift_step;
//		}
//		else if(chassis_set->dir.global_left>Chassis_Move_Y){
//		chassis_set->dir.global_left-=16*shift_step;
//		}
//		else{
//			chassis_set->dir.global_left = Chassis_Move_Y;
//		}

//		if(chassis_set->dir.rotate < Chassis_Move_Z){
//	   chassis_set->dir.rotate+=10*shift_step;
//		}
//		else if(chassis_set->dir.rotate > Chassis_Move_Z){
//		chassis_set->dir.rotate-=16*shift_step;
//		}
//		else{
//		chassis_set->dir.rotate = Chassis_Move_Z;
//		}
//	if(chassis_set->dir.rotate<=8&&chassis_set->dir.rotate>=-8)
//		chassis_set->dir.rotate=0;
//	if(chassis_set->dir.global_front<=8&&chassis_set->dir.global_front>=-8)
//		chassis_set->dir.global_front=0;
//	if(chassis_set->dir.global_left<=8&&chassis_set->dir.global_left>=-8)
//		chassis_set->dir.global_left=0;



//		
//  if(rc_ctrl.rc.sright==RC_MID&&Ctrl_mode==1)
//	{
//		if(chassis_set->chassis_yaw_motor->relative_angle>0.3f || chassis_set->chassis_yaw_motor->relative_angle<-0.3f)
//		{
//				chassis_set->rotate_pid.kp = 5000;
//				chassis_set->dir.rotate = PID_Calculate(&chassis_set->rotate_pid,chassis_set->chassis_yaw_motor->relative_angle,0)+ Chassis_Move_Z;
//		}
//		else
//		{
//                if(chassis_set->chassis_yaw_motor->relative_angle<0.001 && chassis_set->chassis_yaw_motor->relative_angle>-0.001)
//                {
//                    chassis_set->dir.rotate = 0;//死区 
//                }
//                else
//                {
//                    chassis_set->rotate_pid.kp = 7000;
//                    chassis_set->dir.rotate = PID_Calculate(&chassis_set->rotate_pid,chassis_set->chassis_yaw_motor->relative_angle,0)+ Chassis_Move_Z;
//                }
//		}
//		
//		chassis_set->chassis_motor[0].speed_set = chassis_set->dir.global_front - chassis_set->dir.global_left + chassis_set->dir.rotate;
//		chassis_set->chassis_motor[1].speed_set = chassis_set->dir.global_front + chassis_set->dir.global_left + chassis_set->dir.rotate;
//		chassis_set->chassis_motor[2].speed_set = -chassis_set->dir.global_front + chassis_set->dir.global_left + chassis_set->dir.rotate;
//		chassis_set->chassis_motor[3].speed_set = -chassis_set->dir.global_front - chassis_set->dir.global_left + chassis_set->dir.rotate;
//	}
//  else if(rc_ctrl.rc.sright==RC_DOWN||Ctrl_mode==0)
//	{
//        
////		chassis_set->chassis_motor[0].speed_set = chassis_set->dir.global_front - chassis_set->dir.global_left + chassis_set->dir.rotate;
////		chassis_set->chassis_motor[1].speed_set = chassis_set->dir.global_front + chassis_set->dir.global_left + chassis_set->dir.rotate;
////		chassis_set->chassis_motor[2].speed_set = -chassis_set->dir.global_front + chassis_set->dir.global_left + chassis_set->dir.rotate;
////		chassis_set->chassis_motor[3].speed_set = -chassis_set->dir.global_front - chassis_set->dir.global_left + chassis_set->dir.rotate;        

//        
//		Chassis_dirction_trainsfer(chassis_set);		
//		chassis_set->chassis_motor[0].speed_set = chassis_set->dir.local_front + chassis_set->dir.local_left + chassis_set->dir.rotate;
//		chassis_set->chassis_motor[1].speed_set = chassis_set->dir.local_front - chassis_set->dir.local_left + chassis_set->dir.rotate;
//		chassis_set->chassis_motor[2].speed_set = -chassis_set->dir.local_front - chassis_set->dir.local_left + chassis_set->dir.rotate;
//		chassis_set->chassis_motor[3].speed_set = -chassis_set->dir.local_front + chassis_set->dir.local_left + chassis_set->dir.rotate;
//	
//	
//	}

//		float busv=cap->capvolt/800.0f;

//		if(busv<=15.5)
//			{
//		   busV_low_mode=1;
//			 Shift_mode=0;
//			 shift_step=1;
//			 Shift_cnt=0;
//			}
//    else if(busv>=18)
//		  {
//	    busV_low_mode=0;
//	    }

//     if(busV_low_mode==1)
//			 {
//		   for(i=0;i<4;i++)
//				 {
//		      LimitMax(chassis_set->chassis_motor[i].speed_set,2500);
//				 }
//		   }

}

//static void Chassis_PID(Chassis_Control_t *chassis_pid)
//{
//	uint8_t i;
//	if (chassis_pid == NULL)
//    {
//        return;
//    }
//	
//	for (i=0;i<4;i++)
//	{
//		
////		if(SUPER_CAP->chassis_power_buffer_read<=26.5)
////			{
////			chassis_control.chassis_motor[i].speed_set=0.5*chassis_control.chassis_motor[i].speed_set;
////			}
//			
//		
//		PID_Calculate(&chassis_pid->chassis_motor[i].speed_pid, 
//			chassis_pid->chassis_motor[i].speed, chassis_pid->chassis_motor[i].speed_set);
//	}
//}

static void Chassis_sendmessage(Chassis_Control_t *chassis_cmd)
{

 // CAN_CMD_Chassis(60,30,Chassis_Move_X,Chassis_Move_Y,Chassis_Move_Z);

}


static void Chassis_dirction_trainsfer(Chassis_Control_t *chassis_trainsfer)
{
	//
	chassis_trainsfer->dir.M_global_local[0] =  arm_cos_f32(chassis_trainsfer->chassis_yaw_motor->relative_angle);
	chassis_trainsfer->dir.M_global_local[1] = -arm_sin_f32(chassis_trainsfer->chassis_yaw_motor->relative_angle);
	chassis_trainsfer->dir.M_global_local[2] = -arm_sin_f32(chassis_trainsfer->chassis_yaw_motor->relative_angle);
	chassis_trainsfer->dir.M_global_local[3] = -arm_cos_f32(chassis_trainsfer->chassis_yaw_motor->relative_angle);
	
	chassis_trainsfer->dir.local_front = 
		chassis_trainsfer->dir.global_front * chassis_trainsfer->dir.M_global_local[0] +
		chassis_trainsfer->dir.global_left * chassis_trainsfer->dir.M_global_local[1];
	
	chassis_trainsfer->dir.local_left =
		chassis_trainsfer->dir.global_front * chassis_trainsfer->dir.M_global_local[2] +
		chassis_trainsfer->dir.global_left * chassis_trainsfer->dir.M_global_local[3];
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
