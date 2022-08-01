/**
  *******************************************************
  * @file       usart_debug.c/h
  * @brief      串口4的发送数据，用于调试
  * @note       

  @verbatim
  ==============================================================================
  1.底盘调试
	2.云台调试
	3.射击调试
	4.自瞄调试
  ==============================================================================
  @endverbatim
  *******************************************************
  */
 
#include "usart_debug.h"
#include "chassis_task.h"
#include "stm32f4xx_it.h"
#include "swerve_task.h"

extern UART_HandleTypeDef huart6; 

extern uint32_t runningtime_cnt;


void sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    
	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};   
	HAL_UART_Transmit(&huart1, (uint8_t *)cmdf, sizeof(cmdf), 5000);
	HAL_UART_Transmit(&huart1, (uint8_t *)wareaddr, waresize ,5000);
	HAL_UART_Transmit(&huart1, (uint8_t *)cmdr, sizeof(cmdr), 5000);
}

void BT_debug(void)
{
	  float debug_data[8];

/**********蓝牙读取底盘数据********************/	
	  const Chassis_Control_t *chassis;
	  chassis = get_motor_control_point();
//	
  	const motor_measure_t *motor;
	  motor = get_Chassis_Motor_Measure_Point(0);

	const Swerve_Control_t *swerve;
	  swerve = get_swerve_motor_point();

/************************************************************************/



	for(;;)
	{
/*******************chassis_debug***************************************/
//		debug_data[6] = chassis->chassis_motor[0].speed_pid.set; 
//		debug_data[7] = chassis->chassis_motor[0].speed_pid.fdb;
//		debug_data[2] = chassis->chassis_motor[1].speed_pid.set;
//		debug_data[3] = chassis->chassis_motor[1].speed_pid.fdb;
//		debug_data[4] = chassis->chassis_motor[2].speed_pid.set;
//		debug_data[5] = chassis->chassis_motor[2].speed_pid.fdb;
//		debug_data[6] = chassis->chassis_motor[3].speed_pid.set;
//		debug_data[7] = chassis->chassis_motor[3].speed_pid.fdb;	
//		debug_data[1] = chassis->chassis_motor[0].speed_pid.out;
//		debug_data[4] = chassis->chassis_motor[1].speed_pid.out;
//		debug_data[3] = chassis->chassis_motor[1].speed_pid.fdb;
//		debug_data[4] = chassis->chassis_motor[2].speed_pid.set;
//		debug_data[5] = chassis->chassis_motor[2].speed_pid.fdb;
//		debug_data[6] = chassis->chassis_motor[3].speed_pid.set;
//		debug_data[7] = chassis->chassis_motor[3].speed_pid.fdb;

//		debug_data[2] = chassis->chassis_motor->speed_pid.Iout;
//		debug_data[3] = chassis->chassis_motor->speed_pid.out;
//		debug_data[4] = chassis->chassis_motor->speed;

		
//		debug_data[0] = motor->ecd;
//		debug_data[7] = motor[0].speed_rpm;
//		debug_data[2] = motor->given_current;
//		debug_data[3] = motor->temperate;
//		debug_data[4] = motor->last_ecd;
		
/******************************Swerve_debug******************************/

//		debug_data[0] = swerve->swerve_motor[0].gyro_set*1000;
//    debug_data[1] = swerve->swerve_motor[0].gyro*1000;
//	  debug_data[2] = swerve->swerve_motor[0].server_measure[0].speed_rpm;
//    debug_data[3] = swerve->swerve_motor[0].relative_angle_set*1000;
//    debug_data[4] = swerve->swerve_motor[0].relative_angle*1000;
//    debug_data[5] = swerve->swerve_motor[0].relative_angle_algorithm*1000;


		debug_data[2] = swerve->dir.get_dir->Swerve_X;
    debug_data[3] = swerve->dir.get_dir->Swerve_Y;
		debug_data[4] = swerve->dir.get_dir->Swerve_Z;
		debug_data[5] = swerve->dir.local_front;
		debug_data[6] = swerve->dir.local_left;
		debug_data[7] = runningtime_cnt;
//		debug_data[0] = swerve->swerve_motor[0].relative_angle_algorithm*1000;
		debug_data[0] = swerve->swerve_motor[0].relative_angle_set*1000;
		debug_data[1] = swerve->swerve_motor[0].relative_angle*1000;
		
//    debug_data[3] = swerve->chassis_yaw_relative_angle*1000;

//		
//		debug_data[4] = swerve->swerve_motor[0].ecd_algorithm;
//		debug_data[5] = swerve->swerve_motor[0].ecd_fdb;	
//    debug_data[2] = swerve->swerve_motor[2].relative_angle_set;		
//		debug_data[3] = swerve->swerve_motor[2].relative_angle_algorithm;
//    debug_data[4] = swerve->swerve_motor[2].ecd_algorithm;		
//		debug_data[5] = swerve->swerve_motor[2].ecd_fdb;
//		debug_data[6] = swerve->swerve_motor[3].ecd_algorithm;
//		debug_data[7] = swerve->swerve_motor[3].ecd_fdb;
/************************************************************************/
//	//超级电容
//		debug_data[0] =  CAP -> inputpower/100;
//		debug_data[1] =  CAP -> capvolt/800.0f;
//		debug_data[2] =  CAP -> mode;
//		debug_data[3] =  CAP -> setpower/100.0f;	

//		debug_data[4] =  Supercap->chassis_power_read;
//	  debug_data[5] =  Supercap->chassis_power_buffer_read;  
//		debug_data[6] =  Supercap->chassis_power_max;
//if(debug_data[4]==60)
//{
//debug_data[0]=chassis->chassis_motor[0].speed_pid.out
//	             +chassis->chassis_motor[1].speed_pid.out
//	             +chassis->chassis_motor[2].speed_pid.out
//	             +chassis->chassis_motor[3].speed_pid.out;
//}
/************************************************************************/
//视觉数据
  

//    debug_data[4] =  TX -> pitch;
//		debug_data[5] =  TX -> yaw;
//		debug_data[6] =  TX -> aim_delay_time;
//		debug_data[7] =  TX -> aim_sum;

/************************************************************************/
			
			sendware(debug_data,sizeof(debug_data));


		osDelay(1);
	}
}
	


