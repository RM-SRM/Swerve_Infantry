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
    float chassis_power_read;//����ϵͳ��ȡ���̹�����ֵ ��float��
    uint16_t chassis_power_buffer_read;//����ϵͳ��ȡ���̻��幦��ֵ ��uint16��
    uint8_t chassis_power_max;//��ǰ�����������̹��ʣ�float��
    float chassis_power_buffer_max;//��ǰ�����˵��̻��幦������(float)
    
    const supercap_measure_t *supercap ;
	  
    float supercap_power_set;
    float supercap_power_set_delta;//����������Ч����
    float supercap_input_power;
    
    float supercap_power_cmd;//ͨ��CAN�߷��͵Ĺ���
    
    PID_Regulator_t buffer_pid;
}SuperCap_Control_t;

extern SuperCap_Control_t supercap_control;
extern void Supercap_Init(SuperCap_Control_t *supercap_init);
extern const SuperCap_Control_t *get_Supercap_Control_Measure_Point(void);
extern void Supercap_power_read(SuperCap_Control_t *supercap_read);


#endif
