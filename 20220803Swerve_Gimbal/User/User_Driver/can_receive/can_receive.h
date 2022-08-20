#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "can.h"


#define FULLECD 				8192
#define ECDDVALE_MAX 			FULLECD/2					//在1000Hz的采样频率下 转角度不可能超过0.5圈
#define ROLLINGDIRECTION		CLOCKWISE

enum eDIRECTION {ANTICLOCKWISE = +1,CLOCKWISE = -1};

/* CAN send and receive ID */
typedef enum
{
    CAN_YAW_MOTOR_ID = 0x206,
    CAN_SUPERCAP_ID = 0x211,
} can_msg_id_e;

typedef enum
{
	CAN_FRICTION_LEFT_MOTOR_ID=0x201,
  CAN_FRICTION_RIGHT_MOTOR_ID=0x202,
	CAN_RAMMER_ID=0x203,	
	CAN_PITCH_MOTOR_ID=0x205,
} can2_msg_id_e;


//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

//Ecd闭环用结构体
typedef struct
{
	
	motor_measure_t motor_measure;
	int64_t EcdPosition;
	
}motor_ecd_measure_t;

typedef struct
{
    uint16_t inputpower; //输入功率
    uint16_t capvolt;    //电容电压
    uint16_t powermax;   //最大电流
    uint16_t setpower;   //设定功率
}supercap_measure_t;

static uint8_t aData[8];

extern CAN_RxHeaderTypeDef  Rx1Message;



void CAN1_Init(void);
void CAN2_Init(void);
void get_motor_measure(motor_measure_t *motor,uint8_t aData[]);
void get_supercap_measure(supercap_measure_t *cap, uint8_t aData[]);
void get_gimbal_motor_measuer(motor_measure_t *motor,uint8_t aData[]);
void CAN1_Getdata(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[]);
void CAN2_Getdata(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[]);

//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//返回摩擦轮电机变量地址
extern const motor_measure_t *get_Shoot_Motor_Measure_Point(uint8_t i);
//返回拨弹轮电机变量地址
extern const motor_ecd_measure_t *get_Rolling_Motor_Measure_Point(void);
//返回超级电容变量地址
extern const supercap_measure_t *get_SuperCap_Measure_Point(void);


//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern void CAN_CMD_Chassis(uint8_t buffer,uint8_t max_power,int16_t chassis_X,int16_t chassis_Y,int16_t chassis_Z);
//void Lift_motor_output(int16_t iq1,int16_t iq2);
//void CatchPro_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);

extern void CAN_CMD_Shoot(int16_t iq1,int16_t iq2,int16_t iq3);

extern void CAN_CMD_Yaw(int16_t yaw);
extern void CAN_CMD_Pitch(int16_t pitch);
//extern void supercap_sendmessage(float power, uint16_t buffer, uint8_t max_power ,uint8_t command);



#endif
