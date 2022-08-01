#ifndef _SUPERCAPTASK_H
#define _SUPERCAPTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "can_receive.h"
#include "remote.h"
#include "pid.h"
#include "referee.h"

#include "chassis_task.h"


typedef struct
{
    float chassis_power_read;//裁判系统读取底盘功率数值 （float）
    uint16_t chassis_power_buffer_read;//裁判系统读取底盘缓冲功率值 （uint16）
    uint8_t chassis_power_max;//当前机器人最大底盘功率（float）
    float chassis_power_buffer_max;//当前机器人底盘缓冲功率上限(float)
    
    const supercap_measure_t *supercap ;
	  
    float supercap_power_set;
    float supercap_power_set_delta;//缓冲能量等效功率
    float supercap_input_power;
    
    float supercap_power_cmd;//通过CAN线发送的功率
    
    PID_Regulator_t buffer_pid;
}SuperCap_Control_t;

extern SuperCap_Control_t supercap_control;
extern void Supercap_Init(SuperCap_Control_t *supercap_init);
extern const SuperCap_Control_t *get_Supercap_Control_Measure_Point(void);
extern void Supercap_power_read(SuperCap_Control_t *supercap_read);


#endif
