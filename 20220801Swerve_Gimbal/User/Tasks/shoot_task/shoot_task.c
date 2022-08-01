#include "shoot_task.h"
#include "usart_debug.h"
#include "filter.h"
#include "referee.h"
#include "PressTime.h"

/*------------------��ػ�������-------------------*/
#define REVOL_SPEED_RATIO   2160       //�����һ��תһȦ,2160ת��ת��,60*36,����Ƶ�ٳ��Բ��̸����Ϳɵ���Ӧ��Ƶ�µ�ת�� 
#define EIGHT_REVOLVER    1		//8����

#define REVOLVER_CHOOSE  EIGHT_REVOLVER	//ѡ�����ͺ�

#if REVOLVER_CHOOSE == EIGHT_REVOLVER			
	#define		REVOL_SPEED_GRID      8				//���̸���
	#define 	AN_BULLET         (36864.0f)		//�����ӵ����λ������ֵ
#endif
 

 
Shoot_Control_t shoot_control;
static void Button_State(Shoot_Control_t *shoot_set);
static void Shoot_Init(Shoot_Control_t *shoot_init);
static void Shoot_Feedback_Update(Shoot_Control_t *shoot_feedback);
static void Shoot_Set_Control(Shoot_Control_t *shoot_set);
static void Shoot_PID(Shoot_Control_t *shoot_pid);
static void Shoot_Block_TestAndSolve(Shoot_Control_t *shoot_set);
static void Shoot_SingleShoot(int16_t left_heat);
static void Shoot_TriShoot(int16_t left_heat);
static void Shoot_Continue(int16_t left_heat);
static void FeedingRollerPositionReset(void);
void Ecd_Value_Reset_Rolling(void);
Filter_t ShootRoll_filter;
Filter_t corrector_ShootRoll_filter;

volatile int32_t WaitTimeForShoot;
uint8_t robo_level;


extern volatile uint8_t imu_start_dma_flag;

void shoot_task(void)
{
	uint32_t waitetime;
  osDelay(2000);
  Shoot_Init(&shoot_control);
	waitetime = xTaskGetTickCount();
	
	for(;;)
	{
		FeedingRollerPositionReset();
//    Button_State(&shoot_control);
		Shoot_Feedback_Update(&shoot_control);
		Shoot_Set_Control(&shoot_control);
		Shoot_PID(&shoot_control);
		Shoot_Block_TestAndSolve(&shoot_control);
        
        ShootRoll_filter.raw_value = shoot_control.rollingPosition.rollingSpeed.speed_pid.out;
        Chebyshev100HzLPF(&ShootRoll_filter);
        
// 
//        CAN_CMD_Shoot(0,0,0);
        
        CAN_CMD_Shoot(shoot_control.frictionSpeed[0].speed_pid.out
                     ,shoot_control.frictionSpeed[1].speed_pid.out
                     //,ShootRoll_filter.filtered_value);
                     ,shoot_control.rollingPosition.rollingSpeed.speed_pid.out);

		osDelayUntil(&waitetime, SHOOT_TASK_CONTROL_TIME);
		
	}
}

void Shoot_Init(Shoot_Control_t *shoot_init)
{
	if (shoot_init == NULL)
	{
		return ;
	}
	//Ħ���ֵ������ָ���ȡ
	uint8_t i;
	for (i = 0;i < 2 ;i++)
	{
		shoot_init->frictionSpeed[i].rollingMotorMeasure = get_Shoot_Motor_Measure_Point(i);
	}
    //�����ֵ������ָ���ȡ
    shoot_init->rollingPosition.rollingSpeed.rollingMotorMeasure = get_Rolling_Motor_Measure_Point();

	//��������ٶ�λ��pid��ʼ��
	PID_Init(&shoot_init->rollingPosition.position_pid, 			ROLLING_POSITION_PID_MODE,	ROLLING_POSITION_PID_MAX_OUT,
																								ROLLING_POSITION_PID_MAX_IOUT,
																								ROLLING_POSITION_PID_KP,
																								ROLLING_POSITION_PID_KI,
																								ROLLING_POSITION_PID_KD);
 
	PID_Init(&shoot_init->rollingPosition.rollingSpeed.speed_pid, 	ROLLING_SPEED_PID_MODE,		ROLLING_SPEED_PID_MAX_OUT,
																								ROLLING_SPEED_PID_MAX_IOUT,
																								ROLLING_SPEED_PID_KP,
																								ROLLING_SPEED_PID_KI,
																								ROLLING_SPEED_PID_KD);
	//Ħ���ֵ��pid��ʼ��
	for (i = 0;i < 2 ;i++)
	{
		PID_Init(&shoot_init->frictionSpeed[i].speed_pid, 				FRICTION_POSITION_PID_MODE,	FRICTION_POSITION_PID_MAX_OUT,
																								FRICTION_POSITION_PID_MAX_IOUT,
																								FRICTION_POSITION_PID_KP,
																								FRICTION_POSITION_PID_KI,
																								FRICTION_POSITION_PID_KD);
	}
	
	//������������س�ʼ��																	
	shoot_init->rollingPosition.EcdPosition_set = 0;
	shoot_init->rollingPosition.rollingSpeed.speed = 0;
	shoot_init->rollingPosition.rollingSpeed.speed_set = 0;
	
	//Ħ����������س�ʼ��
	for (i = 0;i < 2 ;i++)
	{
		shoot_init->frictionSpeed[i].speed_set = 0;
		shoot_init->frictionSpeed[i].speed     = 0;
	}
	
	//����ṹ��״̬�ж�ö�ٱ�����ʼ��
	shoot_init->blockStage = NOT_BLOCK;
	shoot_init->buttonState = NOT_PRESS;
	shoot_init->fireStatus = NOT_FIRE;
	shoot_init->lastButtonState = NOT_PRESS;
	//����ṹ���ж�ʱ�������ʼ��
	shoot_init->block_time = 0;
	shoot_init->button_press_time = 0;
	//��һ�ε�ecd������ʼ��
	shoot_init->last_ecdset = 0;
}

/*
  ��ȡ��������ٶ�
*/
void Shoot_Feedback_Update(Shoot_Control_t *shoot_feedback)
{
	if (shoot_feedback == NULL)
    {
        return;
    }
	
	uint8_t i;
    
    shoot_feedback->rollingPosition.rollingSpeed.speed = shoot_feedback->rollingPosition.rollingSpeed.rollingMotorMeasure->motor_measure.speed_rpm;
	for (i = 0;i < 2 ;i++)
	{
		shoot_feedback->frictionSpeed[i].speed = shoot_feedback->frictionSpeed[i].rollingMotorMeasure->speed_rpm;
	}
 
	
	
	
	//�ò���Ǩ�Ƶ������жϺ���ȥ��
	//������ecd�ջ���λ������
	//500rpm ��Ӧ����� �����ת��Ϊ36*500 ÿ1/1000�뼫��Ϊ 36*500/60/1000=0.3Ȧ ����ת����ecd�ۼ�Ӧ��С��4096
//	int16_t Ecd_Dvalue = shoot_feedback->rollingPosition.rollingSpeed.rollingMotorMeasure->ecd - shoot_feedback->rollingPosition.rollingSpeed.rollingMotorMeasure->motor_measure.;
//	if (Ecd_Dvalue > ECDDVALE_MAX)
//		Ecd_Dvalue = -FULLECD + Ecd_Dvalue;
//	else if (Ecd_Dvalue < -ECDDVALE_MAX)
//		Ecd_Dvalue = Ecd_Dvalue + FULLECD; 
//	shoot_feedback->rollingPosition.EcdPosition +=  ROLLINGDIRECTION * Ecd_Dvalue;
    
}

/**
 * @Description: �����ַ�תֱ����ת���������ڹ淶�����ֵ�λ�ã���ʼ�е��Ŀ���ֱ���ٳ�ʼ���е��ã�û���ӵ�����Ҫ�ں��ʵĵط�����
 * @Auther: zd
 * @Date: 2021-01-19 14:30:03
 */

static void FeedingRollerPositionReset(void)
{
	//�����λ����� �����ٽ�ѭ��
	if (shoot_control.locationState == LOCDONE)
	{
		return;
	}
	uint16_t delayTime = LOCATION_TIME;
	uint16_t stopCount;
	uint8_t tured = 0;
	for (;delayTime != 0; delayTime--)
	{
		// ����ת������
		Shoot_Feedback_Update(&shoot_control);
		//�ж��Ƿ�ת��
		if (tured  == 0)
		{
			if (shoot_control.rollingPosition.rollingSpeed.speed != 0)
			{
				tured = 1;
			}
		}
		//ת���� �Ż��ж�
		if (tured == 1)
		{
			//����ת�������ж� �Ƿ񿨶�		
			if (shoot_control.rollingPosition.rollingSpeed.speed == 0)
			{
				stopCount++;
			}
			else
			{
				stopCount = 0;
			}
		}
		
		if (stopCount >= LOCATION_WAIT_TIME)
		{
			//�Է���һ ��pid�������
			shoot_control.rollingPosition.position_pid.out = 0;
			shoot_control.rollingPosition.position_pid.Pout = 0;
			shoot_control.rollingPosition.position_pid.Iout = 0;
			shoot_control.rollingPosition.position_pid.Dout = 0;
			shoot_control.rollingPosition.rollingSpeed.speed_pid.out = 0;
			shoot_control.rollingPosition.rollingSpeed.speed_pid.Pout = 0;
			shoot_control.rollingPosition.rollingSpeed.speed_pid.Iout = 0;
			shoot_control.rollingPosition.rollingSpeed.speed_pid.Dout = 0;
			//ecd���
			Ecd_Value_Reset_Rolling();
			shoot_control.rollingPosition.EcdPosition_set = 0;
			shoot_control.locationState = LOCDONE;
			//�˳�����
			return;
		}
		//���С����ת
		float tempOut = - 1500 * ROLLINGDIRECTION;

		osDelay(1);
	}
	Ecd_Value_Reset_Rolling();
	shoot_control.rollingPosition.EcdPosition_set = 0;
	shoot_control.locationState = LOCDONE;
	return;
}







/** 
* �����ֲ������Ŀ���
* @param[in]   	����ṹ��ָ��. 
* @param[out]  	��.      
* @par �޸���־ 
*/
static void Shoot_Set_Control(Shoot_Control_t *shoot_set)
{
	get_robot_shoot_speed_limit(&shoot_set->shoot_speed_max);
	
	

  shoot_set->shoot_left=shoot_set->cooling_Limit-shoot_set->cooling_Data_Now;

  			
 if(rc_ctrl.rc.sleft == RC_UP || rc_ctrl.mouse.press_l == 1){
     switch (shoot_set->shoot_speed_max)
  {
		case 15:
		shoot_set->frictionSpeed[0].speed_set = FRICTIONDIRECTION * 4700 * 1;
	  shoot_set->frictionSpeed[1].speed_set = FRICTIONDIRECTION * 4700 * -1;
		break;
		case 18:
		shoot_set->frictionSpeed[0].speed_set = FRICTIONDIRECTION * 4950 * 1;
	  shoot_set->frictionSpeed[1].speed_set = FRICTIONDIRECTION * 4950 * -1;
		break;
	case 30:
		shoot_set->frictionSpeed[0].speed_set = FRICTIONDIRECTION * 8150 * 1;
	  shoot_set->frictionSpeed[1].speed_set = FRICTIONDIRECTION * 8150 * -1;
	break;
	default:
		break;
  }

 
 
 }

			
          //4640=14.6m/s < 15m/s ��   5100=17.7<18m/s       7600=29<30m/s 


			//aShoot_Continue(500);
            //osDelay(10);
    
 else if(rc_ctrl.rc.sleft == RC_DOWN ||(IF_KEY_PRESSED_V)  )
    {
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1000);
    }
    else
    {
            shoot_set->frictionSpeed[0].speed_set = 0;
	        shoot_set->frictionSpeed[1].speed_set = 0;
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2000);
    }
		
//		shoot_set->frictionSpeed[0].speed_set = FRICTIONDIRECTION * 4650 * 1;
//	  shoot_set->frictionSpeed[1].speed_set = FRICTIONDIRECTION * 4650 * -1;
		
	if(rc_ctrl.rc.sleft==RC_DOWN)
	{
	
	  shoot_set->frictionSpeed[0].speed_set = 0 * 1;
	  shoot_set->frictionSpeed[1].speed_set = 0 * -1;
	}
	switch(shoot_set->fireStatus)
	{
		case NOT_FIRE:
		{
			break;
		}
		case SINGLESHOOT:
		{
            if(WaitTimeForShoot == 3)
            {
                Shoot_SingleShoot(shoot_control.shoot_left);//����ʱ��Ĭ�ϸ�200����shoot_set->cooling_Data_Now
			    shoot_set->fireStatus = NOT_FIRE;
            }
			break;
		}
		case TRISHOOT:
		{
			Shoot_TriShoot(shoot_control.shoot_left);//����ʱ��Ĭ�ϸ�200����
			shoot_set->fireStatus = NOT_FIRE;
			break;
		}
		case CONTINUESHOOT:
		{
            if(WaitTimeForShoot == Shoot_Count-1)
            {
//                Shoot_Continue(shoot_control.shoot_left);//����ʱ��Ĭ�ϸ�200����
							 Shoot_Continue(200);
			    shoot_set->fireStatus = NOT_FIRE;
					
            }
			break;
		}
	}
}
    


void Shoot_PID(Shoot_Control_t *shoot_pid)
{
	if (shoot_pid == NULL)
    {
        return;
    }
	
	uint8_t i;
	//������˫������
	PID_Calculate(&shoot_pid->rollingPosition.position_pid,ROLLINGDIRECTION * shoot_pid->rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition,ROLLINGDIRECTION * shoot_pid->rollingPosition.EcdPosition_set);
	shoot_pid->rollingPosition.rollingSpeed.speed_set = shoot_pid->rollingPosition.position_pid.out;
	PID_Calculate(&shoot_pid->rollingPosition.rollingSpeed.speed_pid,shoot_pid->rollingPosition.rollingSpeed.speed,shoot_pid->rollingPosition.rollingSpeed.speed_set);
	
	//Ħ�����ٶȻ�����
	for(i = 0;i < 2 ;i++)
	{
		PID_Calculate(&shoot_pid->frictionSpeed[i].speed_pid, shoot_pid->frictionSpeed[i].speed,shoot_pid->frictionSpeed[i].speed_set);
	}
}

const Shoot_Control_t *get_shoot_motor_control_point(void)
{
    return &shoot_control;
}

/** 
* ������ ���һ�� 
* ������� �������Ҫ����жϸ����ϸ�
* ����17mm����������� ����42mm����������� ����Ҫ���ԣ�
* @param[in]   	��. 
* @param[out]  	��.      
* @par �޸���־ 
*/
void Shoot_SingleShoot(int16_t left_heat)
{
	#ifdef MM17HEAT
	if (shoot_control.blockStage == NOT_BLOCK && left_heat >= 2*MM17HEAT)
	{
		int32_t Ecd_Diff = shoot_control.rollingPosition.EcdPosition_set - shoot_control.rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition;
		if (Ecd_Diff < SINGLESHOOTECDCOUNT/2)
			shoot_control.rollingPosition.EcdPosition_set += SINGLESHOOTECDCOUNT;
	}
	#elif MM42HEAT
		if (shoot_Control.blockStage == NOT_BLOCK && left_heat >= MM42HEAT)
	{
		int32_t Ecd_Diff = shoot_Control.rollingPosition.EcdPosition_set - shoot_Control.rollingPosition.EcdPosition;
		if (Ecd_Diff < SINGLESHOOTECDCOUNT/2)
			shoot_Control.rollingPosition.EcdPosition_set += SINGLESHOOTECDCOUNT;
	}
	#endif
}

/** 
* ������ �������
* ֻ����17mm�����������
* @param[in]   	��. 
* @param[out]  	��.      
* @par �޸���־ 
*/
void Shoot_TriShoot(int16_t left_heat)
{
	#ifdef MM17HEAT
	int32_t Ecd_Diff = shoot_control.rollingPosition.EcdPosition_set - shoot_control.rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition;
	if (shoot_control.blockStage == NOT_BLOCK && left_heat >= 3 * MM17HEAT)
	{
		if (Ecd_Diff < SINGLESHOOTECDCOUNT/2)
			shoot_control.rollingPosition.EcdPosition_set += 3 * SINGLESHOOTECDCOUNT;
	}
	else
	{
		if (Ecd_Diff < SINGLESHOOTECDCOUNT/2)
		{
			shoot_control.rollingPosition.EcdPosition_set +=  (left_heat / MM17HEAT)* SINGLESHOOTECDCOUNT;
		}
	}
	#endif
}

/** 
* ������ ������� ��Ҫ���ֵ���
* ֻ����17mm�����������
* @param[in]   	��. 
* @param[out]  	��.      
* @par �޸���־ 
*/
void Shoot_Continue(int16_t left_heat)
{
	if (shoot_control.blockStage == NOT_BLOCK && left_heat >= 3*MM17HEAT)
	{
		int32_t Ecd_Diff = shoot_control.rollingPosition.EcdPosition_set - shoot_control.rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition;
		if (Ecd_Diff < SINGLESHOOTECDCOUNT)
			shoot_control.rollingPosition.EcdPosition_set += SINGLESHOOTECDCOUNT;
	}
}
//	else
//	 shoot_control.fireStatus = NOT_FIRE;
	


/** 
* ����״̬�ж�
* @param[in]   	����ṹ��ָ��. 
* @param[out]  	��.      
* @par �޸���־ 
*/
//static void Button_State(Shoot_Control_t *shoot_set)
//{
//	switch(shoot_set->buttonState) 
//	{
//		//��û�а��µ�����£������һ�εİ���״̬ʱ�̰� ��ʱ100ms ������Ǿͽ���һ�εİ���״̬���㣬�����������ģʽ
//		case NOT_PRESS:
//		{
//			if (shoot_set->lastButtonState == SHORT_PRESS)
//				shoot_set->button_press_time ++;
//			if (shoot_set->button_press_time > 200)
//			{
//				shoot_set->lastButtonState = NOT_PRESS;
//				shoot_set->button_press_time = 0;
//			}
//			break;
//		}
//		//�̰�ģʽ�£�����ʱ�䲻����5ms�Ͳ�Ϊ�̰� ����100ms�İ�������Ϊ�Ƕ̰����ٳ��ı���Ϊ�ǳ���
//		case SHORT_PRESS:
//		{
//			if (IF_MOUSE_PRESSED_LEFT)
//			{
//				shoot_set->button_press_time++;
//				if(shoot_set->button_press_time>= 92)
//					shoot_set->buttonState = LONG_PRESS;
//			}
//			else
//			{
////				if (shoot_set->button_press_time >= 1)
////				{
//					shoot_set->fireStatus = SINGLESHOOT;
//					shoot_set->lastButtonState = SHORT_PRESS;
//					shoot_set->buttonState = NOT_PRESS;
//					shoot_set->button_press_time = 0;
////				}
////				else
////				{
////				shoot_set->LastButtonState = NOT_PRESS;
////				shoot_set->ButtonState = NOT_PRESS;
////				shoot_set->button_press_time = 0;
////				}
//			}
//			break;
//		}
//		//����ģʽ ��Ҫ����Ƿ��������ģʽ
//		case LONG_PRESS:
//		{
//			if (shoot_set->lastButtonState == SHORT_PRESS)
//			{
//				shoot_set->buttonState = TRIGGERSHOOTING;
//				shoot_set->button_press_time = 0;
//			}
//			if (IF_MOUSE_PRESSED_LEFT)
//			{
//				
//			}
//			else
//			{
//				shoot_set->fireStatus =  TRISHOOT;
//				shoot_set->buttonState = NOT_PRESS;
//				shoot_set->lastButtonState = LONG_PRESS;
//				shoot_set->button_press_time = 0;
//			}
//			break;
//		}
//		//����ģʽֻ���ɿ���Ż�ֹͣ
//		case TRIGGERSHOOTING:
//		{
//			if (IF_MOUSE_PRESSED_LEFT)
//			{
//				shoot_set->fireStatus = CONTINUESHOOT;
//			}
//			else
//			{
//				shoot_set->buttonState = NOT_PRESS;
//				shoot_set->lastButtonState = TRIGGERSHOOTING;
//				shoot_set->button_press_time = 0;
//			}
//			break;
//		}
//	}
//}

/** 
* ����Ƿ񿨵� ���� ����ת
* @param[in]   �����ṹ��. 
* @param[out]  ��.  
* @par �޸���־ 
*      zd��2020-11-07���� 
*/
static void Shoot_Block_TestAndSolve(Shoot_Control_t *shoot_set)
{
	switch (shoot_set->blockStage)
	{
		case NOT_BLOCK:
		{
			//��Ŀ��ֵ��Ŀǰֵ�ò����1/160 ��ʱ�Żᴥ�������ж�
			if ((shoot_set->rollingPosition.EcdPosition_set - shoot_set->rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition) > SINGLESHOOTECDCOUNT / 160)
			{
				//�������ٶȵ���5ʱ���м���
				if(shoot_set->rollingPosition.rollingSpeed.speed == 0)
				{	
					shoot_set->block_time++;
				}
				else
				{
					shoot_set->block_time = 0;
				}
			}
			else
			{
				shoot_set->block_time = 0;
			}
			//��⵽����ʱ ����50ms ʱ���뿨��״̬
			if (shoot_set->block_time > BLOCKTIMEMAX)
			{
				shoot_set->blockStage = BLOCK;
			}
			shoot_set->last_ecdset = 0;
			break;
		}
		case BLOCK:
		{
			//��Ħ���ֹر� �������ֻ����ڿ���״̬ʱ	˵���Ƕ���1��
			if (shoot_set->fricStaues == CLOSE)
			{
				//����һ��
				shoot_set->rollingPosition.EcdPosition_set = shoot_set->rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition /SINGLESHOOTECDCOUNT * SINGLESHOOTECDCOUNT;
				shoot_set->last_ecdset = 0;
				shoot_set->block_time = 0;
				shoot_set->blockStage = NOT_BLOCK;
				break;
			}
			else
			{
				//���ٺ�֮ǰ�����ֵת��Ϊ������λ�ö�һ�������
				//���ٺ�����ǰ����5ms ��ȫ�������ת10ms 
				
				if (shoot_set->block_time >= BLOCKTIMEMAX
					 && shoot_set->block_time < BLOCKTIMEMAX + FRONTTIME)
				{
					//2500 ��ǰ5ms
					//�������ҵ��趨λ���㹻�ӽ���ʱ�� ���˳�����״̬
					if (shoot_set->rollingPosition.EcdPosition_set
						- shoot_set->rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition < SINGLESHOOTECDCOUNT / 200)
					{
						shoot_set->last_ecdset = 0;
						shoot_set->block_time = 0;
						shoot_set->blockStage = NOT_BLOCK;
						break;
					}
					else
					{
						shoot_set->rollingPosition.rollingSpeed.speed_pid.out = + 2500* ROLLINGDIRECTION;
					}
				}
				//����ת�׶�
				if (shoot_set->block_time >= BLOCKTIMEMAX + FRONTTIME)
				{
					if (shoot_set->last_ecdset == 0)
					{
						//�����ڵ����ֵ���˵���ǰ��λ�õ���һ��
						shoot_set->last_ecdset = (shoot_set->rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition /SINGLESHOOTECDCOUNT + 1) * SINGLESHOOTECDCOUNT ;
						shoot_set->rollingPosition.EcdPosition_set = shoot_set->rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition /SINGLESHOOTECDCOUNT * SINGLESHOOTECDCOUNT;
					}
					else
					{
						//��ת��һ���̶� ȷʵ�ӽ���һ���̶�ʱ �˳�����״̬
						if (shoot_set->rollingPosition.rollingSpeed.rollingMotorMeasure->EcdPosition -  shoot_set->rollingPosition.EcdPosition_set < SINGLESHOOTECDCOUNT / 200)
						{
							shoot_set->rollingPosition.EcdPosition_set = shoot_set->last_ecdset;
							shoot_set->last_ecdset = 0;
							shoot_set->block_time = 0;
							shoot_set->blockStage = NOT_BLOCK;
							break;
						}
						else
						{
							//��תʱ �������5000������һ��ʱ�䣬����ʱ����˳�����״̬
							if (shoot_set->block_time >= BLOCKTIMEMAX + FRONTTIME 
							&& shoot_set->block_time < BLOCKTIMEMAX + FRONTTIME + REVERSETIME)
							{
								shoot_set->rollingPosition.rollingSpeed.speed_pid.out = - 5000 * ROLLINGDIRECTION;
							}
							else
							{
								shoot_set->rollingPosition.EcdPosition_set = shoot_set->last_ecdset;
								shoot_set->last_ecdset = 0;
								shoot_set->block_time = 0;
								shoot_set->blockStage = NOT_BLOCK;
								break;
							}
						}
					}
				}
			}
			
			shoot_set->block_time++;
			if (shoot_set->block_time >= 1000)
			{
				shoot_set->block_time = 1000;
			}
			break;
		}
	}
}


