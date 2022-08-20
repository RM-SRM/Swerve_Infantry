/**
  *******************************************************
  * @file       usart_debug.c/h
  * @brief      ����4�ķ������ݣ����ڵ���
  * @note       

  @verbatim
  ==============================================================================
  1.���̵���
	2.��̨����
	3.�������
	4.�������
  ==============================================================================
  @endverbatim
  *******************************************************
  */
 
#include "usart_debug.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "auto_aim_task.h"
#include "stm32f4xx_it.h"
#include "referee.h"
#include "SupercapTask.h"
#include "Sensor_task.h"

extern UART_HandleTypeDef huart1; 

extern volatile uint8_t finish_flag;


void sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    
	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};   
	HAL_UART_Transmit(&huart1, (uint8_t *)cmdf, sizeof(cmdf), 5000);
	HAL_UART_Transmit(&huart1, (uint8_t *)wareaddr, waresize ,5000);
	HAL_UART_Transmit(&huart1, (uint8_t *)cmdr, sizeof(cmdr), 5000);
}




void BT_task(void)
{
	  float debug_data[8];

/**********������ȡ��������********************/	
//	  const Chassis_Control_t *chassis;
//	  chassis = get_motor_control_point();
//	
//  	const motor_measure_t *motor;
//	  motor = get_Chassis_Motor_Measure_Point(0);


	
	
/**********************************************/
	
/**********������ȡ��̨����********************/
//	const Gimbal_Motor_t *gimbal_pitch;
//    gimbal_pitch = get_pitch_motor_point();
//      const Gimbal_Motor_t *gimbal_yaw;
//	  gimbal_yaw   = get_yaw_motor_point();
/***********************************************/
	
/**********������ȡ��������*******************/
//	const Target_t *aim_X;
//	const Target_t *aim_Y;
// 	aim_X = get_x_autoaim_point();
//	aim_Y = get_y_autoaim_point();
/***********************************************/
	
/**********������ȡ������Ħ��������**************/	
//   const Shoot_Control_t *shoot_motor_t;
//   shoot_motor_t = get_shoot_motor_control_point();
/************************************************/

/**********������ȡ������������************************/
    const supercap_measure_t *CAP;
    CAP = get_SuperCap_Measure_Point();
//		const SuperCap_Control_t *Supercap ;
//		Supercap = get_Supercap_Control_Measure_Point();


/******************************************************/

/***********������ȡ�����Ǵ���������**********************/
//const fp32 *InsAngle;
//InsAngle=get_INS_angle_point();
//const fp32 *Gyro;
//Gyro=get_gyro_data_point();
//const fp32 *Accel;
//Accel=get_gyro_data_point();

// extern bmi088_real_data_t bmi088_real_data;
/********************************************************/

///*****************������ȡMPU�¶ȿ���**************************/
//    const MPU_TEMP *mputempcontrol;
//    mputempcontrol = get_temp_control_point();
/*************************************************************/

///*****************������ȡ����ϵͳ����**************************/
uint16_t chassis_power_max;
get_chassis_maxpower(&chassis_power_max);
float chassis_power;
uint16_t chassis_buffer;
get_chassis_power_and_buffer(&chassis_power,&chassis_buffer);
uint8_t robot_HP;
robot_HP=get_robot_HP();

/*************************************************************/

///*****************��������������**************************/
//  
//    const TX_data_t *TX_test;
//    TX_test = get_TX_data_Measure_Point();
		
/*************************************************************/

  osDelay(1000);

	for(;;)
	{
/*******************chassis_debug***************************************/
//		debug_data[0] = chassis->chassis_motor[0].speed_pid.set;
//		debug_data[1] = chassis->chassis_motor[0].speed_pid.fdb;
//		debug_data[2] = chassis->chassis_motor[1].speed_pid.set;
//		debug_data[3] = chassis->chassis_motor[1].speed_pid.fdb;
//		debug_data[4] = chassis->chassis_motor[2].speed_pid.set;
//		debug_data[5] = chassis->chassis_motor[2].speed_pid.fdb;
//		debug_data[6] = chassis->chassis_motor[3].speed_pid.set;
//		debug_data[7] = chassis->chassis_motor[3].speed_pid.fdb;

//		debug_data[2] = chassis->chassis_motor->speed_pid.Iout;
//		debug_data[3] = chassis->chassis_motor->speed_pid.out;
//		debug_data[4] = chassis->chassis_motor->speed;

		
//		debug_data[0] = motor->ecd;
//		debug_data[1] = motor->speed_rpm;
//		debug_data[2] = motor->given_current;
//		debug_data[3] = motor->temperate;
//		debug_data[4] = motor->last_ecd;
		
		
//		//��ȡ����
//		debug_data[5] = GetPower_10mW(&hi2c2);
/***********************************************************************/
		
/***********************gimbal_debug***********************************/
		
		//pitch yaw offset ecd 
//		debug_data[0] = gimbal_pitch->gimbal_motor_measure->ecd;
//		debug_data[1] = gimbal_yaw->gimbal_motor_measure->ecd;
		
//		debug_data[0] = gimbal_pitch->gyro*1000;
//		debug_data[1] = gimbal_yaw->gyro*1000;
//		debug_data[2] = gimbal_pitch->gimbal_motor_measure->ecd;
//		debug_data[3] = gimbal_yaw->gimbal_motor_measure->ecd;
//		debug_data[4] = gimbal_pitch->absolute_angle*1000;
//      debug_data[5] = gimbal_yaw->absolute_angle*1000;			
		
  //pitch���ٶȻ�
		//�ٶȻ����趨ֵ����ֵ�ĵ�λ���ǻ���,debugʱ�Ŵ�1000���۲�
//		debug_data[0] = gimbal_pitch->gyro_set*1000;
//		debug_data[1] = gimbal_pitch->gyro*1000;
//		debug_data[2] = gimbal_pitch->speed_pid.Iout;
//		debug_data[3] = gimbal_pitch->speed_pid.out;
//		debug_data[4] = gimbal_pitch->gimbal_motor_measure->ecd;
		
		//pitch��ǶȻ�
//		debug_data[3] = gimbal_pitch->angle_pid.set*1000;
//		debug_data[4] = gimbal_pitch->angle_pid.fdb*1000;
//		debug_data[2] = gimbal_pitch->angle_pid.Iout;
//		debug_data[5] = gimbal_pitch->angle_pid.out;
//        debug_data[4] = gimbal_pitch->absolute_angle;


//	//yaw��
  		//�ٶȻ����趨ֵ����ֵ�ĵ�λ���ǻ���,debugʱ�Ŵ�1000���۲�
//		debug_data[0] = gimbal_yaw->speed_pid.set*1000;
//		debug_data[1] = gimbal_yaw->speed_pid.fdb*1000;
//		debug_data[2] = gimbal_yaw->speed_pid.Iout;
//		debug_data[3] = gimbal_yaw->speed_pid.out;
		//yaw��ǶȻ�
//		debug_data[2] = gimbal_yaw->angle_pid.set*1000;
//		debug_data[3] = gimbal_yaw->angle_pid.fdb*1000;
//		debug_data[2] = gimbal_yaw->angle_pid.Iout;
//		debug_data[3] = gimbal_yaw->angle_pid.out;	
/*********************************************************************/
		
/**************shoot_debug********************************************/
//     //Ħ���� 1 3508
//     debug_data[0] = - (shoot_motor_t->frictionSpeed[0].speed_set);
//     debug_data[1] = - (shoot_motor_t->frictionSpeed[0].speed_pid.fdb);
//	   debug_data[2] = shoot_motor_t->frictionSpeed[0].speed_pid.Iout;
//		 debug_data[3] = shoot_motor_t->frictionSpeed[0].speed_pid.out;
//		 //Ħ���� 2 3508
//     debug_data[2] = shoot_motor_t->frictionSpeed[1].speed_set;
//     debug_data[3] = shoot_motor_t->frictionSpeed[1].speed_pid.fdb;
//		 debug_data[2] = shoot_motor_t->shoot_motor[1].speed_pid.Iout;
//		 debug_data[3] = shoot_motor_t->shoot_motor[1].speed_pid.out;
//     //������ 3 2006
//		 debug_data[0] = shoot_motor_t->shoot_motor[2].speed_pid.set;
//     debug_data[1] = shoot_motor_t->shoot_motor[2].speed_pid.fdb;
//		 debug_data[2] = shoot_motor_t->shoot_motor[2].speed_pid.Iout;
//		 debug_data[3] = shoot_motor_t->shoot_motor[2].speed_pid.out;
//       debug_data[7] = (shoot_motor_t->shoot_left);


//     debug_data[0] = shoot_motor_t->shoot_speed_max;
//     debug_data[1] = shoot_motor_t->bullet_speed;
/**********************************************************************/
	
/*************************auto_aim_debug********************************/
    //����ȷ��bֵ->Aim_B
//		debug_data[0] = aim_X->initial;
//		debug_data[1] = aim_Y->initial;
//		
//		debug_data[2] = aim.x.process_angle;
//		debug_data[3] = aim.y.process_angle;
		
		//debug_data[4] = camera_flag;
		
/***********************************************************************/
//���µ��̲���
//      
//      debug_data[0] = gimbal_pitch->relative_angle*1000;
/************************************************************************/


/************************************************************************/
//���̸�����̨����
//        debug_data[0] = chassis->chassis_yaw_motor->relative_angle;
//        debug_data[1] = chassis->rotate_pid.set*1000;
//        debug_data[2] = chassis->rotate_pid.fdb*1000;
//        debug_data[3] = chassis->rotate_pid.out*1000;

//        debug_data[3] = CAP->inputvolt/100.f;
//        debug_data[4] = CAP->capvolt/100.f;
//        debug_data[5] = CAP->inputcurrent/100.f;
//        debug_data[6] = CAP->setpower/100.f;
//        


//            get_chassis_power_and_buffer(&debug_data[0],&debug_data[1]);
//            get_chassis_maxpower(&debug_data[2]);
//          debug_data[3] = CAP->inputvolt;
//          debug_data[4] = CAP->capvolt;
//          debug_data[5] = CAP->inputcurrent;
//          debug_data[6] = CAP->setpower;
//          debug_data[7] = finish_flag;
/************************************************************************/

/**********************������ȡ����������*********************************/
//			debug_data[0] = InsAngle[0]*1000;
//			debug_data[1] = InsAngle[1]*1000;
//			debug_data[2] = InsAngle[2]*1000;
//			debug_data[7] = bmi088_real_data.temp;
//			debug_data[7] =sensor_temp;
//      debug_data[3] = temperature;
//			debug_data[3] = Gyro[0];
//			debug_data[4] = Gyro[1];
//			debug_data[5] = Gyro[2];

//			debug_data[3] = Accel[0];
//			debug_data[4] = Accel[1];
//		debug_data[5] = Accel[2];
//        debug_data[3] = MPUtemp[0];
//        debug_data[4] = mputempcontrol->temp_set;
//        debug_data[5] = mputempcontrol->temp_pid.fdb;
//        debug_data[6] = mputempcontrol->temp_pid.out;
/************************************************************************/
	//��������
//		debug_data[0] =  CAP -> inputpower/100;
//		debug_data[1] =  CAP -> capvolt/1000;
//		debug_data[2] =  CAP -> powermax;
//		debug_data[3] =  CAP -> setpower/100;	
//		
//		debug_data[4] =  Supercap->chassis_power_read;
//	  debug_data[5] =  Supercap->chassis_power_buffer_read;     
//		debug_data[6] =  Supercap->chassis_power_max;

/************************************************************************/
	//����ϵͳ
get_chassis_maxpower(&chassis_power_max);
get_chassis_power_and_buffer(&chassis_power,&chassis_buffer);
robot_HP=get_robot_HP();


    debug_data[0] =  chassis_power_max;
		debug_data[1] =  chassis_power;
		debug_data[2] =  chassis_buffer;
		debug_data[3] =  robot_HP;
		debug_data[4] =  CAP -> capvolt/1000.0f;

/************************************************************************/
//�Ӿ�����
 

//    debug_data[0] =  TX_test -> aim_x;
//		debug_data[1] =  TX_test -> aim_y;
//		debug_data[2] =  TX_test -> aim_delay_time;
//		debug_data[3] =  TX_test -> aim_sum;




		
/************************************************************************/
			
			sendware(debug_data,sizeof(debug_data));


		osDelay(1);
	}
}
	


