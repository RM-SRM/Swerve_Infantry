#ifndef _PRESSTIME_H
#define _PRESSTIME_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "remote.h"
#include "pid.h"
#include "tim.h"
#include "remote.h"
#include "shoot_task.h"

#define  Enable_Count 5
#define  Shoot_Count 30

extern volatile int32_t WaitTimeForShoot;

extern uint8_t R_mode;
extern uint8_t Q_mode;
extern uint8_t E_mode;
extern uint8_t G_mode;
extern uint8_t F_mode;
extern uint8_t X_mode;
extern uint8_t Z_mode;
extern uint8_t V_mode;
extern uint8_t Ctrl_mode;
extern uint8_t Shift_mode;
extern uint8_t shift_step;
extern int Q_cnt;
extern int E_cnt;
extern int Shift_cnt;
#endif

