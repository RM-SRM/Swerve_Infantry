#include "gimbal_task.h"
#include "filter.h"
#include "arm_math.h"
#include "usart.h"
#include "referee.h"
#include "Sensor_Task.h"
#include "PressTime.h"
#include "INFANTRY_picture.h"

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

#define JSCOPE_WATCH_gimbal 0
#if JSCOPE_WATCH_gimbal
//j-scope ����pid����
static void Jscope_Watch_gimbal(void);
#endif
	
#define SYSTEM_IDENTIFICATION 0
#if SYSTEM_IDENTIFICATION
uint16_t point_all,t;
float once_lenth,x,y;
float fs[]={
4.0, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0,
5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 
10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 
14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0, 17.5, 
18.0, 18.5, 19.0, 19.5, 20.0, 20.5, 21.0, 21.5,
22.0,24, 26, 28, 30, 32, 34, 36, 38,50, 60, 70, 
80, 90, 100, 110, 120, 200, 250, 333};

#endif 

Gimbal_Control_t gimbal_control;
Filter_t gimbal_pitch_out_filter;
Filter_t gimbal_yaw_out_filter;
Filter_t corrector_yaw_speed,corrector_yaw_position;
Filter_t Filter_mouse_x;
Filter_t Filter_mouse_y;

#define GIMBALCMD 1

static void Gimbal_Init(Gimbal_Control_t *gimbal_init);
static void Gimbal_Position_Reset(Gimbal_Control_t *gimbal_pos_reset);
static void Gimbal_Feedback_Update(Gimbal_Control_t *gimbal_feedback_updata);
static float Motor_Absolute_to_Relative_Change(float angle,float offset);
static float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd);
static void Gimbal_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
static void Gimbal_PID(Gimbal_Control_t *gimbal_pid);
static float Gimbal_AnglePID_Calculate(PID_Regulator_t *pid , float fdb, float set, float DELTA);
static int16_t get_abs(int16_t value);
  

uint8_t gimbal_reset_flag=0;//���ڴ�����״̬��Ѹ���˳�

uint8_t T_aim;

int16_t time=0;

int16_t G_blocktime=0;


extern volatile uint8_t imu_start_dma_flag;
void gimbal_task(void const * argument)
{
	
	uint32_t waittime;

	

	//��̨�����ʼ��
	Gimbal_Init(&gimbal_control);
	osDelay(100);	
    
while(imu_start_dma_flag != 1)//|| mpu6500.temperature < TEMP_SET-0.2f)
    {
    Gimbal_Feedback_Update(&gimbal_control);
		osDelay(1);
    }
						//�ȴ���������Ʈ�����ռ����
	
#if GIMBALCMD == 1
//	Gimbal_Position_Reset(&gimbal_control);
#endif	
    
	waittime = xTaskGetTickCount();

	for (;;)
	{
		
		
#if SYSTEM_IDENTIFICATION
		for (i=0;i< (sizeof(fs) / 4);i++)
		{
		point_all = 20000/fs[i];
		once_lenth = fs[i]*2*PI/1000;
		t=0;
		x=0;
		while(t<point_all)
		{
			
			y =	arm_sin_f32(x)*2;
//			if (arm_sin_f32(x)>0) y=2;
//			if (arm_sin_f32(x)<0) y=-2;
			if (i==0 ) y=0;
			x += once_lenth;
#endif 
		Gimbal_Feedback_Update(&gimbal_control);
		Gimbal_Set_Contorl(&gimbal_control);
		
		Gimbal_PID(&gimbal_control);

		gimbal_pitch_out_filter.raw_value = gimbal_control.pitch_motor.speed_pid.out;
		gimbal_yaw_out_filter.raw_value = gimbal_control.yaw_motor.speed_pid.out;//y * 3000;
		Chebyshev100HzLPF(&gimbal_yaw_out_filter);
		Chebyshev100HzLPF(&gimbal_pitch_out_filter);

#if GIMBALCMD == 1


		
		    if(rc_ctrl.rc.sright==RC_DOWN&&rc_ctrl.rc.sleft==RC_DOWN)
					{
               CAN_CMD_Yaw(0);
               CAN_CMD_Pitch(0);
					}
        else
					{
		          CAN_CMD_Yaw(gimbal_yaw_out_filter.filtered_value);
              CAN_CMD_Pitch(gimbal_pitch_out_filter.filtered_value);
          }
#endif	
    
#if JSCOPE_WATCH_gimbal		
		Jscope_Watch_gimbal();
#endif
//		osDelay(1);
		osDelayUntil(&waittime, GIMBAL_TASK_CONTROL_TIME);
	}
#if SYSTEM_IDENTIFICATION
}}
#endif 
//}//daihuijiushan
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
	return &gimbal_control.yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
	return &gimbal_control.pitch_motor;
}

static void Gimbal_Init(Gimbal_Control_t *gimbal_init)
{
	//�������ָ���ȡ
	gimbal_init->pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	gimbal_init->yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	//����������ָ���ȡ
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_gyro_data_point();
	//ң��������ָ���ȡ
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	//��������ָ���ȡ
	gimbal_init->yaw_motor.target = get_x_autoaim_point();
	gimbal_init->pitch_motor.target = get_y_autoaim_point();
	//yaw���PID��ʼ��
	PID_Init(&gimbal_init->yaw_motor.speed_pid, YAW_SPEED_PID_MODE,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT,YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD);
	PID_Init(&gimbal_init->yaw_motor.angle_pid, YAW_ANGLE_PID_MODE,YAW_ANGLE_PID_MAX_OUT,YAW_ANGLE_PID_MAX_IOUT,YAW_ANGLE_PID_KP,YAW_ANGLE_PID_KI,YAW_ANGLE_PID_KD);
	//pitch���PID��ʼ��
	PID_Init(&gimbal_init->pitch_motor.speed_pid, PITCH_SPEED_PID_MODE,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD);
	PID_Init(&gimbal_init->pitch_motor.angle_pid, PITCH_ANGLE_PID_MODE,PITCH_ANGLE_PID_MAX_OUT,PITCH_ANGLE_PID_MAX_IOUT,PITCH_ANGLE_PID_KP,PITCH_ANGLE_PID_KI,PITCH_ANGLE_PID_KD);

	gimbal_init->pitch_motor.offset_AbToRe = 0;		//��ת��offset��0
	gimbal_init->yaw_motor.offset_AbToRe = 0;
	//��̨�����ֵ��ʼ��
	gimbal_init->pitch_motor.offset_ecd = PITCH_OFFSET_ECD;
	gimbal_init->yaw_motor.offset_ecd = YAW_OFFSET_ECD;

}

//���λ���������ã�ÿ����̨�ϵ�֮ǰִ��һ��
static void Gimbal_Position_Reset(Gimbal_Control_t *gimbal_pos_reset)
{
	uint32_t waitetime;
	
	waitetime = xTaskGetTickCount();
	
	gimbal_pos_reset->pitch_motor.offset_AbToRe = 0;		//��ת��offset��0
	gimbal_pos_reset->yaw_motor.offset_AbToRe = 0;
	
	while(int_abs( (gimbal_pos_reset->pitch_motor.gimbal_motor_measure->ecd
					-gimbal_pos_reset->pitch_motor.offset_ecd) ) > 80  )					//pitch���û��̧��ˮƽλ��ʱִ��
	{
		//��������������̨̧����ˮƽλ��
		Gimbal_Feedback_Update(gimbal_pos_reset);
		gimbal_pos_reset->pitch_motor.relative_angle_set = -0;
		
						
		Gimbal_PID(&gimbal_control);
		gimbal_pitch_out_filter.raw_value = gimbal_control.pitch_motor.speed_pid.out;
		
		Chebyshev100HzLPF(&gimbal_pitch_out_filter);

		CAN_CMD_Pitch(gimbal_pitch_out_filter.filtered_value);
	
		osDelayUntil(&waitetime, GIMBAL_TASK_CONTROL_TIME);
	}
	
	
	//pitch����ˮƽλ�ú����������ǲ�����yaw��pitch�ǣ�����ת��offset
	gimbal_pos_reset->pitch_motor.offset_AbToRe = gimbal_pos_reset->pitch_motor.absolute_angle
													-gimbal_pos_reset->pitch_motor.relative_angle;
	
	gimbal_pos_reset->yaw_motor.offset_AbToRe = gimbal_pos_reset->yaw_motor.absolute_angle
													-gimbal_pos_reset->yaw_motor.relative_angle;
  
	gimbal_pos_reset->yaw_motor.absolute_angle_set = gimbal_pos_reset->yaw_motor.absolute_angle;

}

static void Gimbal_Feedback_Update(Gimbal_Control_t *gimbal_feedback)
{
	if (gimbal_feedback == NULL)
    {
        return;
    }
	//��̨���ٶ����ݸ���
	gimbal_feedback->pitch_motor.gyro = -*(gimbal_feedback->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);
	gimbal_feedback->yaw_motor.gyro = *(gimbal_feedback->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
	//�����ǽǶ����ݸ���
	gimbal_feedback->pitch_motor.absolute_angle = *(gimbal_feedback->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
	gimbal_feedback->pitch_motor.absolute_angle = Motor_Absolute_to_Relative_Change(gimbal_feedback->pitch_motor.absolute_angle ,
																					gimbal_feedback->pitch_motor.offset_AbToRe);
	gimbal_feedback->yaw_motor.absolute_angle = *(gimbal_feedback->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	gimbal_feedback->yaw_motor.absolute_angle = Motor_Absolute_to_Relative_Change(gimbal_feedback->yaw_motor.absolute_angle ,
																					gimbal_feedback->yaw_motor.offset_AbToRe);
	//��̨�Ƕ����ݸ���

	gimbal_feedback->pitch_motor.relative_angle = Motor_ecd_to_angle_Change(gimbal_feedback->pitch_motor.gimbal_motor_measure->ecd,
																			gimbal_feedback->pitch_motor.offset_ecd);
	gimbal_feedback->yaw_motor.relative_angle = Motor_ecd_to_angle_Change(gimbal_feedback->yaw_motor.gimbal_motor_measure->ecd,
																		  gimbal_feedback->yaw_motor.offset_ecd);
 
}

//�����ԽǶȺ���ԽǶ�����
static float Motor_Absolute_to_Relative_Change(float angle,float offset)
{
	angle -= offset;
	if (angle>PI) angle -= 2*PI;
	if (angle<-PI) angle += 2*PI;
	return angle;
}


//��������ˮƽ��ǰ����Ϊ��̨��������
static float Motor_ecd_to_angle_Change(uint16_t ecd, uint16_t offset_ecd) 
{
	//���㵱ǰλ�������Ĳ�
	int32_t relative_ecd = ecd - offset_ecd;
	
	//����ֵ����4096���򽫽����ȥ8192��˵�������ʱ����㸺һ��
    if (relative_ecd > HALF_RANGE)
    {
        relative_ecd -= FULL_RANGE;
    }
	//����ֵС��-4096���򽫽������8192��˵�������ʱ�������һ��
    else if (relative_ecd < -HALF_RANGE)
    {
        relative_ecd += FULL_RANGE;
    }
	
    return relative_ecd * Motor_Ecd_to_Rad;
}

static void Gimbal_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
	
	
	int16_t Gimbal_Yaw;//yaw����
	int16_t Gimbal_Pitch;//pitch����


	Gimbal_Pitch = -gimbal_set_control->gimbal_rc_ctrl->rc.ch[1];
	Gimbal_Yaw = gimbal_set_control->gimbal_rc_ctrl->rc.ch[0];
	
	if(rc_ctrl.mouse.x !=0 || rc_ctrl.mouse.y != 0 )
	{
		Filter_mouse_x.raw_value=rc_ctrl.mouse.x * 3.0;
  	Filter_mouse_y.raw_value=rc_ctrl.mouse.y * 3.0;
		Chebyshev100HzLPF(&Filter_mouse_x);
		Chebyshev100HzLPF(&Filter_mouse_y);
		
		Gimbal_Yaw += Filter_mouse_x.filtered_value;
		Gimbal_Pitch -= Filter_mouse_x.filtered_value;
	}
			 

	
	//ң��������δ����ʱ��pitch����ˮƽ��yaw����״̬
	if (gimbal_set_control->gimbal_rc_ctrl->rc.sright == 0) //|| gimbal_set_control->gimbal_rc_ctrl->rc.sright == RC_DOWN)
	{
		gimbal_set_control->pitch_motor.relative_angle_set = -0;
		gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle;
	}
	
	//���Ͽ��ز����м䣬�ص��м�
	if ((gimbal_set_control->gimbal_rc_ctrl->rc.sright == RC_MID ||gimbal_set_control->gimbal_rc_ctrl->rc.sright == RC_DOWN)) //&& gimbal_set_control->gimbal_rc_ctrl->rc.sleft == RC_MID)
	{
//		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
        
    gimbal_set_control->pitch_motor.absolute_angle_set = gimbal_set_control->pitch_motor.absolute_angle_set - Gimbal_Pitch/200000.0;
		LimitMaxAngle(gimbal_set_control->pitch_motor.absolute_angle_set, 0.4f,0.45f);
		gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle_set - Gimbal_Yaw/100000.0;
        
		if (gimbal_set_control->yaw_motor.absolute_angle_set > PI) gimbal_set_control->yaw_motor.absolute_angle_set -=2*PI;
		if (gimbal_set_control->yaw_motor.absolute_angle_set <-PI) gimbal_set_control->yaw_motor.absolute_angle_set +=2*PI;
	}
	else if((gimbal_set_control->gimbal_rc_ctrl->rc.sright == RC_MID ||gimbal_set_control->gimbal_rc_ctrl->rc.sright == RC_DOWN) && gimbal_set_control->gimbal_rc_ctrl->rc.sleft == RC_UP)
	{
//		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
//		gimbal_set_control->pitch_motor.absolute_angle_set = gimbal_set_control->pitch_motor.absolute_angle_set;
//		LimitMax(gimbal_set_control->pitch_motor.absolute_angle_set, 0.4f);
//		gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle_set;
        
    gimbal_set_control->pitch_motor.absolute_angle_set = gimbal_set_control->pitch_motor.absolute_angle_set - Gimbal_Pitch/200000.0;
		LimitMaxAngle(gimbal_set_control->pitch_motor.absolute_angle_set, 0.4f,0.45f);
		gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle_set - Gimbal_Yaw/100000.0;

		if (gimbal_set_control->yaw_motor.absolute_angle_set > PI) gimbal_set_control->yaw_motor.absolute_angle_set -=2*PI;
		if (gimbal_set_control->yaw_motor.absolute_angle_set <-PI) gimbal_set_control->yaw_motor.absolute_angle_set +=2*PI;
	}
	
	if((gimbal_set_control->gimbal_rc_ctrl->rc.sright == RC_UP || IF_MOUSE_PRESSED_RIGHT)&&camera_flag==1)
	{
//		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
//		gimbal_set_control->pitch_motor.absolute_angle_set = gimbal_set_control->pitch_motor.absolute_angle - aim.y.angle[0];
//        gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle - aim.x.angle[0];
        gimbal_set_control->pitch_motor.absolute_angle_set = gimbal_set_control->pitch_motor.absolute_angle - aim.y.angle[0];
		    LimitMaxAngle(gimbal_set_control->pitch_motor.absolute_angle_set, 0.4f,0.45f);
		    gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle-aim.x.angle[0];
//        gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle - aim.x.filted_angle- Gimbal_Yaw/10000.0;
        camera_flag=0;
		    gimbal_reset_flag=1;
		if(aim.x.angle[0]==0&&aim.y.angle[0]==0&&TX_data.aim_delay_time==AIM_MISSED){//�Ӿ�������װ�װ�
		  G_blocktime++;
			if(G_blocktime>=100){
			gimbal_set_control->pitch_motor.absolute_angle_set = 0;
			}
		}
		else{
			 G_blocktime=0;
		}
//      gimbal_set_control->pitch_motor.absolute_angle_set = gimbal_set_control->pitch_motor.absolute_angle - aim.y.process_angle;
//		gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle - aim.x.process_angle;
	}
   if(gimbal_reset_flag==1)
	 {
	   if(gimbal_set_control->gimbal_rc_ctrl->rc.sright == RC_UP || IF_MOUSE_PRESSED_RIGHT)//����������״̬
		 {
		 }
	   else//���Ѵ�����״̬�˳�
		 {
     gimbal_set_control->pitch_motor.absolute_angle_set = gimbal_set_control->pitch_motor.absolute_angle - Gimbal_Pitch/200000.0;//�ӵ�ǰ�Ƕ�λ�ÿ�ʼ�˶�,��ֹ�������Ժ��޷��ָ�
		 LimitMaxAngle(gimbal_set_control->pitch_motor.absolute_angle_set, 0.4f,0.45f);
		 gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle - Gimbal_Yaw/100000.0;
        
		 if (gimbal_set_control->yaw_motor.absolute_angle_set > PI) gimbal_set_control->yaw_motor.absolute_angle_set -=2*PI;
		 if (gimbal_set_control->yaw_motor.absolute_angle_set <-PI) gimbal_set_control->yaw_motor.absolute_angle_set +=2*PI;	
		 gimbal_reset_flag=0;
		 }
	 }
//	if((gimbal_set_control->gimbal_rc_ctrl->rc.sright == RC_UP&& gimbal_set_control->gimbal_rc_ctrl->rc.sleft == RC_UP))
//	{
//		
//        gimbal_set_control->pitch_motor.absolute_angle_set = gimbal_set_control->pitch_motor.absolute_angle - aim.y.angle[0];
//        gimbal_set_control->yaw_motor.absolute_angle_set = gimbal_set_control->yaw_motor.absolute_angle- Gimbal_Yaw/100000.0;

//		
//	}
		    if(rc_ctrl.rc.sright==RC_DOWN&&rc_ctrl.rc.sleft==RC_DOWN){
        gimbal_set_control->yaw_motor.absolute_angle_set=gimbal_set_control->yaw_motor.absolute_angle;
				}
}

static void Gimbal_PID(Gimbal_Control_t *gimbal_pid)
{
	//pitch 
	
	
	Gimbal_AnglePID_Calculate(&gimbal_pid->pitch_motor.angle_pid , gimbal_pid->pitch_motor.absolute_angle,
	gimbal_pid->pitch_motor.absolute_angle_set, gimbal_pid->pitch_motor.gyro);
	gimbal_pid->pitch_motor.gyro_set = -gimbal_pid->pitch_motor.angle_pid.out;
	PID_Calculate(&gimbal_pid->pitch_motor.speed_pid, gimbal_pid->pitch_motor.gyro, gimbal_pid->pitch_motor.gyro_set);
	
	
	//pitch -angle debug
//	time++;
//	if (time>1000)
//		time=-1000;
//	if(time>=0)  {
//	gimbal_pid->pitch_motor.absolute_angle_set = 0.3;}
//	else if(time<0) {
//	gimbal_pid->pitch_motor.absolute_angle_set = -0.3;}

//	Gimbal_AnglePID_Calculate(&gimbal_pid->pitch_motor.angle_pid , gimbal_pid->pitch_motor.absolute_angle,
//		gimbal_pid->pitch_motor.absolute_angle_set, gimbal_pid->pitch_motor.gyro);
//	gimbal_pid->pitch_motor.gyro_set = gimbal_pid->pitch_motor.angle_pid.out;

//		PID_Calculate(&gimbal_pid->pitch_motor.speed_pid, gimbal_pid->pitch_motor.gyro, gimbal_pid->pitch_motor.gyro_set);
//  
	//pitch -speed debug
//	if(gimbal_pid->pitch_motor.gimbal_motor_measure->ecd<=2900&&gimbal_pid->pitch_motor.gimbal_motor_measure->ecd>=2600)  
//	gimbal_pid->pitch_motor.gyro_set = 0.5;
//	if(gimbal_pid->pitch_motor.gimbal_motor_measure->ecd>=4000&&gimbal_pid->pitch_motor.gimbal_motor_measure->ecd<=4300)
//	gimbal_pid->pitch_motor.gyro_set = -0.5;
//	
//	  PID_Calculate(&gimbal_pid->pitch_motor.speed_pid, gimbal_pid->pitch_motor.gyro, gimbal_pid->pitch_motor.gyro_set);


//yaw 
#if SYSTEM_IDENTIFICATION
	gimbal_pid->yaw_motor.relative_angle_set = 0-gimbal_pid->yaw_motor.relative_angle;//(float)(gimbal_control.gimbal_rc_ctrl->rc.ch[1])/500.
	Gimbal_AnglePID_Calculate(&gimbal_pid->yaw_motor.angle_pid , 0, gimbal_pid->yaw_motor.relative_angle_set, gimbal_pid->yaw_motor.gyro);
	
	corrector_yaw_speed.raw_value = gimbal_pid->yaw_motor.gyro-y;
	Corrector_Yaw_Speed(&corrector_yaw_speed);
	//gimbal_pid->yaw_motor.gyro_set = gimbal_pid->yaw_motor.gyro-y;
	gimbal_pid->yaw_motor.gyro_set = corrector_yaw_speed.filtered_value;
	PID_Calculate(&gimbal_pid->yaw_motor.speed_pid, gimbal_pid->yaw_motor.gyro_set, 0);
#else	

//  time++;
//	if (time>1400)
//		time=-1400;
//	if(time>=0)  {
//	gimbal_pid->yaw_motor.absolute_angle_set = 0.3;}
//	else if(time<0) {
//	gimbal_pid->yaw_motor.absolute_angle_set = -0.7;}
    
		//΢С��Χ��������
    if(gimbal_pid->yaw_motor.absolute_angle_set - gimbal_pid->yaw_motor.absolute_angle <= 0.0001f && 
    gimbal_pid->yaw_motor.absolute_angle_set - gimbal_pid->yaw_motor.absolute_angle >= -0.0001f)
    {
        gimbal_pid->yaw_motor.absolute_angle_set = gimbal_pid->yaw_motor.absolute_angle;
    }
		
    Gimbal_AnglePID_Calculate(&gimbal_pid->yaw_motor.angle_pid , gimbal_pid->yaw_motor.absolute_angle, gimbal_pid->yaw_motor.absolute_angle_set, gimbal_pid->yaw_motor.gyro);
    corrector_yaw_speed.raw_value = gimbal_pid->yaw_motor.angle_pid.out - gimbal_pid->yaw_motor.gyro;//-gimbal_pid->yaw_motor.gyro;//
    Corrector_Yaw_Speed(&corrector_yaw_speed);
    gimbal_pid->yaw_motor.gyro_set = gimbal_pid->yaw_motor.gyro-gimbal_pid->yaw_motor.angle_pid.out;
    //	gimbal_pid->yaw_motor.gyro_set = -corrector_yaw_speed.filtered_value;
    PID_Calculate(&gimbal_pid->yaw_motor.speed_pid, gimbal_pid->yaw_motor.gyro_set, 0);
	
	////yaw debug	
	
//	if(gimbal_pid->yaw_motor.gimbal_motor_measure->ecd<=3000&&gimbal_pid->yaw_motor.gimbal_motor_measure->ecd>=2800)  
//	gimbal_pid->yaw_motor.gyro_set = -2.0;
//	if(gimbal_pid->yaw_motor.gimbal_motor_measure->ecd>=500&&gimbal_pid->yaw_motor.gimbal_motor_measure->ecd<=1000)
//	gimbal_pid->yaw_motor.gyro_set = 2.0;
//	
//	PID_Calculate(&gimbal_pid->yaw_motor.speed_pid, gimbal_pid->yaw_motor.gyro, gimbal_pid->yaw_motor.gyro_set);
	
	
#endif

}

static float Gimbal_AnglePID_Calculate(PID_Regulator_t *pid , float fdb, float set, float DELTA)
{
	if (pid == NULL)
    {
        return 0.0f;
    }
	
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];

	pid->set = set + PI;
	pid->fdb = fdb + PI;
	//С���ݱ�Ҫ����
	if (pid->set - pid->fdb > PI)
		pid->err[0] = pid->set - pid->fdb -2*PI;
	else if  (pid->set - pid->fdb < -PI)
		pid->err[0] = pid->set - pid->fdb +2*PI;
	else pid->err[0] = pid->set - pid->fdb;
	
	if (pid->mode == PID_POSITION)
	{
		pid->Pout = pid->kp * pid->err[0];
		pid->Iout += pid->ki * pid->err[0];
		pid->Dout = pid->kd *DELTA;
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	else if (pid->mode == PID_DELTA)
	{
		pid->Pout = pid->kp * (pid->err[0] - pid->err[1]);
		pid->Iout = pid->ki * pid->err[0];
		pid->Dout = pid->kd * DELTA;
		pid->out += pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	return pid->out;
	
}

static int16_t get_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}



#if JSCOPE_WATCH_gimbal
float jlook_y,jlook_gyro;

static void Jscope_Watch_gimbal(void)
{
	jlook_y = y;
	jlook_gyro = gimbal_control.yaw_motor.gyro;

}
#endif
