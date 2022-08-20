#ifndef AUTO_AIM_TASK_H
#define AUTO_AIM_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "remote.h"



#define TX2_X_HIGHOFFSET 2
#define TX2_X_LOWOFFSET 3
#define TX2_Y_HIGHOFFSET 5
#define TX2_Y_LOWOFFSET 6


//×ÔÃébÖµ
#define Aim_B 1350

typedef struct
{
	int16_t initial;
	int16_t relative;
	float angle[2];
	float imu_gyro[2];
	float aimspeed;
	float accl;
	float filted_angle;
	float filted_speed;
    float process_angle;	
} Target_t;

typedef struct
{
	uint8_t data[9];
	uint8_t flag_end;
	char check;

} TX2_t;

typedef struct
{
	
	int mode_ch;//10=armar_stable,20=armaer=rotating,30=energy_stable,40=energy_rotating
	int board_size;//11=small,21=big
	int base_directly;//12=infantry,22=hero,32=base,42=sentry
	int color;//13=red,23=blue
	float bullet_speed_max;
	float insquat[4];


} Target_Mode;


typedef struct
{
	float yaw;
	float pitch;
	float aim_delay_time;
	float aim_sum;
}TX_data_t;

typedef struct
{
	uint32_t last_time;
  uint32_t new_time;
	Target_t x,y;
	const float *gimbal_gyro_point;
	uint8_t Ts;
	const RC_ctrl_t *aim_rc_ctrl;
	TX2_t TX2;
	
} Aim_t;

extern uint8_t TX2_data[1];
extern Aim_t aim;
extern void auto_aim_task(void const * argument);
 void huoqu (uint8_t* Buf);
extern uint8_t camera_flag;
extern void Auto_data(void);
//extern void Getdata_Camera(void);
extern const Target_t *get_y_autoaim_point(void);
extern const Target_t *get_x_autoaim_point(void);
extern const TX_data_t  *get_TX_data_Measure_Point(void);
extern const Target_Mode *get_target_mode_point(void);
extern TX_data_t TX_data;
#endif
