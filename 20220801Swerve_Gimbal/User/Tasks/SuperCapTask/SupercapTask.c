#include "SupercapTask.h"



void Supercap_power_read(SuperCap_Control_t *supercap_read);
void Supercap_Init(SuperCap_Control_t *supercap_init);
void Supercap_Feedback_Update(SuperCap_Control_t *supercap_feedback);
void Supercap_Set_Control(SuperCap_Control_t *supercap_set);

SuperCap_Control_t supercap_control;

static uint16_t powermax;
uint8_t data[2];



void supercap_task(void)
{
    //计数器cnt LED8 1Hz闪烁
    uint16_t cnt = 0;
    
    //User SuperCap Code Start Before Loop
    
    Supercap_Init(&supercap_control);
    osDelay(1000);
    //User SuperCap Code End   Before Loop
    while(1)
    {
			


		
        if(cnt == 499)
        {
					  cnt=0;
            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
           
        }
			  		
				 
				cnt++;
        osDelay(1);
    }
}








//读取底盘功率
void Supercap_power_read(SuperCap_Control_t *supercap_read)
{
	   
    get_chassis_power_and_buffer(&supercap_read->chassis_power_read,&supercap_read->chassis_power_buffer_read);
    get_chassis_maxpower(&powermax);
    supercap_read->chassis_power_max=(uint8_t)powermax ;
    //后续需要增加飞坡状态读取
    //正常底盘缓冲能量60J
    //飞坡底盘缓冲能量250J
    if(1)
    {
        supercap_read->chassis_power_buffer_max = 60;
    }
}

//超级电容初始化
void Supercap_Init(SuperCap_Control_t *supercap_init)
{
    if(supercap_init == NULL)
    {
        return;
    }
    
    //获取超级电容CAN线信息指针
    supercap_init->supercap = get_SuperCap_Measure_Point();
    //获取初始状态底盘信息
		
    get_chassis_power_and_buffer(&supercap_init->chassis_power_read,&supercap_init->chassis_power_buffer_read);
    get_chassis_maxpower(&powermax);
		supercap_init->chassis_power_max=(uint8_t)(powermax);
    //底盘最大缓冲能量初始设定
    supercap_init->chassis_power_buffer_max = 60;
    //设置初始底盘缓冲能量等效功率为0
    supercap_init->supercap_power_set_delta = 0;
    
}


const SuperCap_Control_t *get_Supercap_Control_Measure_Point(void)
{
    return &supercap_control;
}
