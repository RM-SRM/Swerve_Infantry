#include "auto_aim_task.h"
#include "Sensor_task.h"
#include "kalman_filter.h"
#include "usart.h"
#include <stdlib.h>
#include "gimbal_task.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "usb_device.h"
#include "referee.h"
#include "PressTime.h"
#include "fifo_float.h"

#define int_abs(x) ((x) > 0 ? (x) : (-x))

//任务周期，单位ms
#define AURO_AIM_TASK_CONTROL 5//视觉需求200hz

#define JSCOPE_WATCH_aim 1
#if JSCOPE_WATCH_aim
//j-scope 帮助pid调参
static void Jscope_Watch_gimbal(void);
float jlook_yrelative,jlook_yinitial,jlook_yangle;
float jlook_follow;
#endif

#define INTERNALTEST 0
#if INTERNALTEST
uint16_t point_all,t;
float once_lenth,x,y,p;
float fs[]={
4.0,1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0,
5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 
10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 
14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0, 17.5, 
18.0, 18.5, 19.0, 19.5, 20.0, 20.5, 21.0, 21.5,
22.0,24, 26, 28, 30, 32, 34, 36, 38,50, 60, 70, 
80, 90, 100, 110, 120, 200, 250, 333};
#endif 

static void Aim_Init(Aim_t *aim_init, kalman_filter_init_t *I);
static void Aim_Feedback_Update(Aim_t *aim);
static float Aim_Position_to_Angle(int16_t Pos);

static void AutoAim_INFO_PROCESS(void);

static BYTEQueue_t target_yaw_angel_queue,target_yaw_gyro_queue,target_pitch_angel_queue,target_pitch_gyro_queue;
static kalman_filter_t F_x,F_y;
static kalman_filter_init_t I;
Aim_t aim;
TX_data_t TX_data;

uint8_t Auto_Buffer[16];
Target_Mode target_mode;
uint8_t TX2_data[1];

static float target_yaw_angel_buffer[4];
static float target_yaw_gyro_buffer[4];
static float target_pitch_angel_buffer[4];
static float target_pitch_gyro_buffer[4];
static void kalman_filter_feedback(kalman_filter_t *F);
extern volatile uint8_t imu_start_dma_flag;

uint8_t id;
uint16_t bullet_speed_limit;
uint8_t camera_flag = 0;
#define TXINFOCNT 8

void auto_aim_task(void const * argument)
{
	uint32_t waitetime;

	osDelay(1000);
	//初始化各项参数、指针, 定义kalman矩阵
	Aim_Init(&aim, &I);

	//HAL_UART_Receive_DMA(&huart8,Auto_Buffer,10);
	
	//初始化kalman矩阵,用相同的I矩阵
	kalman_filter_init(&F_x, &I);
	kalman_filter_init(&F_y, &I);
	while(imu_start_dma_flag != 1)//|| mpu6500.temperature < TEMP_SET-0.2f)
   {
	osDelay(1);
   }
	osDelay(2000);
	waitetime = xTaskGetTickCount();
	for(;;)
	{
		
#if INTERNALTEST
		for (i=0;i< (sizeof(fs) / 4);i++)
		{
		point_all = 20000/fs[i];
		once_lenth = fs[i]*2*PI/1000;
		t=0;
		x=0;
		while(t<point_all)
		{
			p= rand()%100 -0; 
			y =	arm_sin_f32(x)*2 + p/1000-0.05;
//			if (arm_sin_f32(x)>0) y=2;
//			if (arm_sin_f32(x)<0) y=-2;
			if (i==0 ) y=0;
			x += once_lenth;
			t++;
#endif
		
		Aim_Feedback_Update(&aim);
		//卡尔曼滤波
    kalman_filter_feedback(&F_x);
		kalman_filter_feedback(&F_y);
    kalman_filter_calc(&F_x, aim.x.angle[0], aim.x.aimspeed, aim.x.accl,aim.x.accl);//应该把测量量转化为世界坐标系
		kalman_filter_calc(&F_y, aim.y.angle[0], aim.y.aimspeed, aim.y.accl,aim.y.accl);
    //做判断
	if ( (F_x.filtered_value[0] * F_x.filtered_value[1] >0) && (int_abs(aim.x.relative) >0.02) )
	{
        aim.x.filted_angle = F_x.filtered_value[0] +F_x.filtered_value[1] *aim.Ts * 0.03f;
	}
    else aim.x.filted_angle = F_x.filtered_value[0];
    


//		kalman_filter_calc(&F_x, aim.x.angle[0],aim.x.speed,aim.x.accl,aim.x.accl);
//		kalman_filter_calc(&F_y, aim.y.angle[0],aim.y.speed,aim.y.accl,aim.y.accl);
//		if ( (F_x.filtered_value[0] * F_x.filtered_value[1] >0) && (int_abs(aim.x.relative) >0.02) )
//		{
//			aim.x.filted_angle = aim.x.angle[0]+ F_x.filtered_value[1]*AURO_AIM_CONTROL_TIME*0.03f;
//		}
//		else aim.x.filted_angle = F_x.filtered_value[0];
//		     aim.x.filted_speed = F_x.filtered_value[1];
//		     aim.y.filted_angle = F_y.filtered_value[0];
//		     aim.y.filted_speed = F_y.filtered_value[1];

//	  HAL_UART_Transmit(&huart8,(uint8_t*)&aa,sizeof(aa),55);   
//    HAL_UART_Transmit(&huart8,(uint8_t*)&target_mode,sizeof(Target_Mode),55);  

//		CDC_Transmit_FS((uint8_t*)&aa,sizeof(aa));
		CDC_Transmit_FS((uint8_t*)&target_mode,36);

		
#if JSCOPE_WATCH_aim		
		Jscope_Watch_gimbal();
#endif

		osDelayUntil(&waitetime, AURO_AIM_TASK_CONTROL);
	}
	
	
#if INTERNALTEST
}}
#endif 
}

void huoqu (uint8_t* Buf)
{
	memcpy(Auto_Buffer,Buf,16);
  Auto_data();
}

const Target_t *get_x_autoaim_point(void)
{
	return &aim.x;
}

const Target_t *get_y_autoaim_point(void)
{
	return &aim.y;
}

const Target_Mode *get_target_mode_point(void)
{
	return &target_mode;
}
static void Aim_Init(Aim_t *aim_init, kalman_filter_init_t *I)
{
	//差分队列初始化
	 QueueInit_f(&target_yaw_angel_queue, target_yaw_angel_buffer, 4);
   QueueInit_f(&target_yaw_gyro_queue,  target_yaw_gyro_buffer, 4);
	 QueueInit_f(&target_pitch_angel_queue, target_pitch_angel_buffer, 4);
   QueueInit_f(&target_pitch_gyro_queue,  target_pitch_gyro_buffer, 4);
	//陀螺仪数据指针获取
	aim_init->gimbal_gyro_point = get_gyro_data_point();
	//delta t
	aim_init->Ts = AURO_AIM_TASK_CONTROL/1000.0;
	//遥控器数据指针获取
	aim_init->aim_rc_ctrl = get_remote_control_point();
	//设定kalman矩阵
	I->A_data[0] = 1; I->A_data[1] = aim_init->Ts;
	I->A_data[2] = 0; I->A_data[3] = 1;
	
	I->B_data[0] =  pow(aim.Ts,2)/2; I->B_data[1] = aim_init->Ts;
	I->B_data[2] = 0; I->B_data[2] = 1;
	
	I->Q_data[0] = 0.001; I->Q_data[1] = 0;
	I->Q_data[2] = 0; I->Q_data[3] = 0.001;
	
	I->R_data[0] = 0.001; I->R_data[1] = 0;
	I->R_data[2] = 0; I->R_data[3] = 0.001;
	
	I->H_data[0] = 1; I->H_data[1] = 0; 
	I->H_data[2] = 0; I->H_data[3] = 1;
//	HAL_UART_Receive_DMA(&huart8,TX2_data,1);
}

static void Aim_Feedback_Update(Aim_t *aim_feedback)
{
	//角速度存储
	aim_feedback->x.imu_gyro[1] = aim_feedback->x.imu_gyro[0];
	aim_feedback->y.imu_gyro[1] = aim_feedback->y.imu_gyro[0];
	//角速度更新
	aim_feedback->x.imu_gyro[0] = *(aim_feedback->gimbal_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
	aim_feedback->y.imu_gyro[0] = -*(aim_feedback->gimbal_gyro_point +INS_GYRO_X_ADDRESS_OFFSET);
//	//算出速度增量
//	aim_feedback->x.accl = aim_feedback->x.imu_gyro[0] - aim_feedback->x.imu_gyro[1];
//	aim_feedback->y.accl = aim_feedback->y.imu_gyro[0] - aim_feedback->y.imu_gyro[1];

    //连续TXINFOCNT组中值滤波
    //AutoAim_INFO_PROCESS();
	//获取相对中心的坐标
//	if (aim_feedback->x.initial == 0 && aim_feedback->y.initial == 0)
//	{
//		aim_feedback->x.relative = 0;
//		aim_feedback->y.relative = 0;
//	}
//	else 
//	{
//		aim_feedback->x.relative = aim_feedback->x.initial - 320;
//		aim_feedback->y.relative = aim_feedback->y.initial - 240;

//	}
//	//将非线性位置数据转为角度，变成线性数据
//	aim_feedback->x.angle[1] = aim_feedback->x.angle[0]; 
//	aim_feedback->y.angle[1] = aim_feedback->y.angle[0];

//#if INTERNALTEST
//	aim_feedback->x.angle[0] = y;
//#endif
//	aim_feedback->x.speed = (aim_feedback->x.angle[0] - aim_feedback->x.angle[1])/aim_feedback->Ts;
//	aim_feedback->y.speed = (aim_feedback->x.angle[0] - aim_feedback->x.angle[1])/aim_feedback->Ts;


//读取小电脑所需数据
	const Gimbal_Motor_t *pit;
   pit = get_pitch_motor_point();
  const Gimbal_Motor_t *yaw;
	 yaw  = get_yaw_motor_point();
	const fp32 *Insquat;
	Insquat = get_INS_quat_point();
	id=get_robot_id();
  if(id>100){target_mode.color=23;}
	if(id<100){target_mode.color=13;}
	target_mode.insquat[0]=Insquat[0];
  target_mode.insquat[1]=Insquat[1];
	target_mode.insquat[2]=Insquat[2];
	target_mode.insquat[3]=Insquat[3];
  if(G_mode==0)
	  target_mode.mode_ch=10;
	else if(G_mode==1)
		target_mode.mode_ch=30;
	else if(G_mode==2)
		target_mode.mode_ch=40;
target_mode.board_size=11;
target_mode.base_directly=22;
get_robot_shoot_speed_limit(&bullet_speed_limit);
if(bullet_speed_limit==0)bullet_speed_limit=15;//在裁判系统离线时保证视觉那边有弹速数据
target_mode.bullet_speed_max=bullet_speed_limit;
}	
  
static float Aim_Position_to_Angle(int16_t pos)
{
	float angle;
	angle = atan((1.0*pos) /Aim_B);

//	if (int_abs(pos) > 300) angle = atan((1.0*pos) /1500);
//	else if (int_abs(pos) > 100) angle = atan((1.0*pos) /1400);
//	else angle = atan((1.0*pos) /1400)*0.01*int_abs(pos)*0.01*int_abs(pos);

	return angle;
}




void Auto_data(void)
{
	camera_flag=1;
  aim.x.angle[1] = aim.x.angle[0]; 
	aim.y.angle[1] = aim.y.angle[0];

  memcpy(&TX_data,Auto_Buffer,16);
    	
	aim.last_time=aim.new_time;
	aim.new_time=osKernelSysTick();
	aim.Ts=aim.new_time-aim.last_time;
	if(TX_data.aim_sum-TX_data.aim_delay_time-TX_data.pitch-TX_data.yaw<=0.002)
	{
	  aim.x.angle[0] = TX_data.yaw;
		aim.y.angle[0] = TX_data.pitch;
		
    aim.x.aimspeed = QueueGetDiff_f(&target_yaw_angel_queue, aim.x.angle[0]) / aim.Ts;
		aim.x.accl = QueueGetDiff_f(&target_yaw_gyro_queue, aim.x.aimspeed) / aim.Ts;
    aim.y.aimspeed = QueueGetDiff_f(&target_pitch_angel_queue, aim.y.angle[0]) / aim.Ts;
		aim.y.accl = QueueGetDiff_f(&target_pitch_gyro_queue, aim.y.aimspeed) / aim.Ts;

	}
    
	else
	{
		
	  aim.x.angle[0]=0;
	
    aim.y.angle[0]=0;
	
	}

}




const TX_data_t  *get_TX_data_Measure_Point(void)
{
    return &TX_data;
}


static void kalman_filter_feedback(kalman_filter_t *F) {
    F->A.pData[1] = aim.Ts;
    F->B.pData[0] = pow(aim.Ts,2)/2;
    F->B.pData[1] = aim.Ts;
}




//void Getdata_Camera(void)
//{
//	switch (aim.TX2.flag_end)
//	{
//		case 0:
//			aim.TX2.check =0;
//			if (TX2_data[0]=='$')
//				aim.TX2.flag_end=1;
//			else 
//				aim.TX2.flag_end=0;
//		break;
//		
//		case 1:
//			aim.TX2.check =0;
//			if (TX2_data[0]==' ')
//				aim.TX2.flag_end=2;
//			else 
//				aim.TX2.flag_end=0;
//		break;
//			
//		case 2:
//			aim.TX2.check =0;
//			if (TX2_data[0]==' ')
//				aim.TX2.flag_end=3;
//			else 
//				aim.TX2.flag_end=0;
//		break;	

//		case 3:
//			aim.TX2.check =0;
//			if (TX2_data[0]==' ')
//				aim.TX2.flag_end=4;
//			else 
//				aim.TX2.flag_end=0;
//		break;		
//		
//		case 4:
//			aim.TX2.check =  TX2_data[0];
//			aim.x.initial &= 0x00ff;
//			aim.x.initial |= (TX2_data[0]<<8);
//			aim.TX2.flag_end=5;
//		break;
//		
//		case 6:
//			aim.x.initial &= 0xff00;
//			aim.TX2.check += TX2_data[0];
//			aim.x.initial |= TX2_data[0];
//			aim.TX2.flag_end=7;
//		break;
//	
//		case 7:
//			aim.TX2.check += TX2_data[0];
//			aim.y.initial &= 0x00ff;
//			aim.y.initial |= (TX2_data[0]<<8);
//			aim.TX2.flag_end=8;
//		break;
//		
//		case 8:
//			aim.y.initial &= 0xff00;
//			aim.TX2.check += TX2_data[0];
//			aim.y.initial |= TX2_data[0];
//			aim.TX2.flag_end=0;
//		
//			if((aim.x.initial>640)||(aim.x.initial<=0)){
//	    aim.x.initial=320;
//    	}
//      if((aim.y.initial>480)||(aim.y.initial<=0)){
//      aim.y.initial=240;
//    	}
//			
//			break;
//	
//	}
//	
//}

//全局变量 使用容器对收到的视觉数据进行滤波处理
//volatile float docker_x[4]={320,320,320,320};
//volatile float docker_y[4]={240,240,240,240};
int docker_x = 320,docker_y = 240;
volatile int aimdatacnt_x=0;
volatile int aimdatacnt_y=0;
//取连续三组数据进行中值滤波
void AutoAim_INFO_PROCESS(void)
{    
    if(aim.x.initial == 320)
    {
        aimdatacnt_x ++;
    }
    else
    {
        aimdatacnt_x = 0;
        docker_x = aim.x.initial;
    }
    
    if(aim.y.initial == 240)
    {
        aimdatacnt_y ++;
    }
    else
    {
        aimdatacnt_y = 0;
        docker_y = aim.y.initial;
    }
    
    if(aimdatacnt_x >= TXINFOCNT)
    {
        aimdatacnt_x = 0;
        docker_x = 320;
    }

    if(aimdatacnt_y >= TXINFOCNT)
    {
        aimdatacnt_y = 0;
        docker_y = 240;
    }
    
    aim.x.initial = docker_x;
    aim.y.initial = docker_y;
    
    
//    int i;
//    float process_x = 0;
//    float process_y = 0;
//    
//    docker_x[2] = docker_x[1];
//    docker_y[2] = docker_y[1];
//    docker_x[1] = docker_x[0];
//    docker_y[1] = docker_y[0];
//    
//    docker_x[0] = aim.x.initial;
//    docker_y[0] = aim.y.initial;
//    
//    for(i=0;i<3;i++)
//    {
//        process_x += docker_x[i];
//        process_y += docker_y[i];
//    }
//    
//    aim.x.process_angle = process_x/3;
//    aim.y.process_angle = process_y/3;
}






#if JSCOPE_WATCH_aim

static void Jscope_Watch_gimbal(void)
{

	jlook_yrelative = aim.y.relative;
	jlook_yinitial = aim.y.initial;
	jlook_yangle = aim.y.angle[0];
	
}
#endif
