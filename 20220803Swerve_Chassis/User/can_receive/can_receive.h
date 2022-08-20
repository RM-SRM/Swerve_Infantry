#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "can.h"


#define FULLECD 				8192
#define HALFECD         FULLECD/2
#define QUATERECD       FULLECD/4
#define ECDDVALE_MAX 			FULLECD/2					//在1000Hz的采样频率下 转角度不可能超过0.5圈
#define ROLLINGDIRECTION		CLOCKWISE

enum eDIRECTION {ANTICLOCKWISE = +1,CLOCKWISE = -1};


/* CAN send and receive ID */
typedef enum
{
    CAN_6020_M1_ID = 0x205,
    CAN_6020_M2_ID = 0x206,
    CAN_6020_M3_ID = 0x207,
    CAN_6020_M4_ID = 0x208,


   
} can2_msg_id_e;

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
	  CAN_3508_M4_ID = 0x204,
  	CAN_YAW_MOTOR_ID = 0x206,
	
	  CAN_SUPERCAP_ID = 0x211,
	  SWERVE_CMD_ID =  0x210,
} can1_msg_id_e;


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
	  uint16_t mode;    
//    uint16_t powermax;   //最大电流
    uint16_t setpower;   //设定功率
}supercap_measure_t;

typedef struct
{
    int16_t Swerve_X;
    int16_t Swerve_Y;    
	  int16_t Swerve_Z;   

}Swerve_direction_t;

static uint8_t aData[8];

extern CAN_RxHeaderTypeDef  Rx1Message;



void CAN1_Init(void);
void CAN2_Init(void);
void get_motor_measure(motor_measure_t *motor,uint8_t aData[]);
void get_swerve_measure(Swerve_direction_t *dir,uint8_t aData[]);
void get_supercap_measure(supercap_measure_t *cap, uint8_t aData[]);
void get_gimbal_motor_measuer(motor_measure_t *motor,uint8_t aData[]);
void CAN1_Getdata(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[]);
void CAN2_Getdata(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[]);

//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//返回摩擦轮电机变量地址
extern const motor_measure_t *get_Shoot_Motor_Measure_Point(uint8_t i);
//返回拨弹轮电机变量地址
extern const motor_ecd_measure_t *get_Rolling_Motor_Measure_Point(void);
//返回超级电容变量地址
extern const supercap_measure_t *get_SuperCap_Measure_Point(void);
//返回舵轮电机变量地址
extern const motor_measure_t *get_Swerve_Gimbal_Motor_Measure_Point(uint8_t i);
//返回底盘运动方向变量地址
extern const Swerve_direction_t *get_Swerve_Direction_Point(void);
//返回底盘最大功率地址
extern const uint8_t *get_Chassis_PowerMax_Point(void);
//返回底盘缓冲能量地址
extern const int16_t *get_Chassis_Buffer_Point(void);


//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern void CAN_CMD_Chassis(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
//void Lift_motor_output(int16_t iq1,int16_t iq2);
//void CatchPro_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);

extern void CAN_CMD_Shoot(int16_t iq1,int16_t iq2,int16_t iq3);
extern void CAN_CMD_Swerve(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
//extern void CAN_CMD_Yaw(int16_t yaw);
extern void CAN_CMD_Pitch(int16_t pitch);
//extern void supercap_sendmessage(float power, uint16_t buffer, uint8_t max_power ,uint8_t command);



#endif
