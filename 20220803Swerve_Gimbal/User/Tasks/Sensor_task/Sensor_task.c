/**
  *******************************************************
  * @file       Sensor_task.c/h
  * @brief      主要利用陀螺仪mpu6500，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过mpu6500的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间，提供注释对应的宏定义，关闭DMA，
  * 
  * @note      当前A板安装方式Angle与轴对应关系
  *             |----Angle----|-----轴------|
  *             |      0      |     YAW     |
  *             |      1      |     ROLL    |
  *             |      2      |     PITCH   |
  *             |-------------|-------------|
  *            当前A板安装方式与Gyro对应关系
  *             |----GYRO-----|-----轴------|
  *             |      0      |     PITCH   |
  *             |      1      |     ROLL    |
  *             |      2      |     YAW     |
  *             |-------------|-------------|
  *            当前A板安装方式与Accel对应关系
  *             |----Accel----|-----轴------|
  *             |      0      |     ROLL    |
  *             |      1      |     PITCH   |
  *             |      2      |     YAW     |
  *             |-------------|-------------|
  * 
  *       
Sensor_task
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *******************************************************
  */

#include "Sensor_Task.h"
#include "bsp_flash.h"
#include "string.h"
#include "bsp_adc.h"
#include "BMI088Middleware.h"
#include "pid.h"
#include "can_receive.h"

static head_cali_t  head_cali;       //head cali data


/* USER FUNCTION VOID BEGIN*/
void updata_imu_temperature(void);
int8_t get_control_temperature(void);
static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch);

void bmi088Offset_Cal(bmi088_real_data_t *bmiOffSetCal);
void bmi088OffSet(void);
void GetOffsetFromFlash(float *  accOFFSet,float * gyroOFFSet,float * magOFFset);
void WriteOffsetFromFlash(float accOFFSet[3],float gyroOFFSet[3],float magOFFset[3]);
void Offset_Getdata(void);
/* USER FUNCTION VOID END*/


#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \
    




/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          Ðý×ªÍÓÂÝÒÇ,¼ÓËÙ¶È¼ÆºÍ´ÅÁ¦¼Æ,²¢¼ÆËãÁãÆ¯,ÒòÎªÉè±¸ÓÐ²»Í¬°²×°·½Ê½
  * @param[out]     gyro: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[out]     accel: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[out]     mag: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[in]      bmi088: ÍÓÂÝÒÇºÍ¼ÓËÙ¶È¼ÆÊý¾Ý
  * @param[in]      ist8310: ´ÅÁ¦¼ÆÊý¾Ý
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ¿ØÖÆbmi088µÄÎÂ¶È
  * @param[in]      temp:bmi088µÄÎÂ¶È
  * @retval         none
  */
static void imu_temp_control(fp32 temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ¸ù¾Ýimu_update_flagµÄÖµ¿ªÆôSPI DMA
  * @param[in]      temp:bmi088µÄÎÂ¶È
  * @retval         none
  */

static void imu_cmd_spi_dma(void);

extern SPI_HandleTypeDef hspi1;


static TaskHandle_t SensorTask_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
bmi088_real_data_t bmi088_off_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
//´æ´¢ÍÓÂÝÒÇ½ÇËÙ¶ÈÁãÆ¯Êý¾Ý±äÁ¿
fp32 gyro_offset[3];
//ÍÓÂÝÒÇÊý¾Ý¼ÆËã´æ´¢±äÁ¿
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static PID_Regulator_t imu_temp_pid;

static const float timing_time = 0.001f;   //tast run time , unit s.ÈÎÎñÔËÐÐµÄÊ±¼ä µ¥Î» s


uint16_t imu_tempctl_tim=0;
//ÓÃÓÚÍÓÂÝÒÇÁãÆ¯Ð£Õý
imu_cali_t local_cali_t;

//¼ÓËÙ¶È¼ÆµÍÍ¨ÂË²¨
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};




static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.Å·À­½Ç µ¥Î» rad

//¸ÃÊý¾ÝÓÃÓÚ½ÃÕý
float accOFFSet[3] 	= {0,0,0};
float gyroOFFSet[3]	= {0,0,0};
float magOFFSet[3]	= {0,0,0};

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imuÈÎÎñ, ³õÊ¼»¯ bmi088, ist8310, ¼ÆËãÅ·À­½Ç
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void Sensor_task(void const *pvParameters)
{
    static uint16_t count_time = 0;
    
    //wait a time
    osDelay(INS_TASK_INIT_TIME);
    
    //陀螺仪磁力计初始化
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(ist8310_init())
    {
        osDelay(100);
    }

    //在未达到设定温度前不进行零漂处理
    //陀螺仪升温
    //1.初始化陀螺仪温度控制PID
    PID_init_IMU(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    //2.陀螺仪持续升温
    while(bmi088_real_data.temp < IMU_TEMP_CONTROL_SET - 2.0f)
    {
        BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
        imu_temp_control(bmi088_real_data.temp);
    		imu_tempctl_tim++;
				if(imu_tempctl_tim==1000)
				{
					imu_tempctl_tim=0;
					HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
				}
			  CAN_CMD_Chassis(50,30,15000,15000,15000);	
		}
    
    
    //零漂数据计算 在ZERODRIFT_TIMES时间内累加求和
    //1.进行零漂计算
//    while(count_time < ZERODRIFT_TIMES)
//    {
//        BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
//       INS_gyro[0] = bmi088_real_data.gyro[0];
//        INS_gyro[1] = bmi088_real_data.gyro[1];
//        INS_gyro[2] = bmi088_real_data.gyro[2];
//        INS_cali_gyro(local_cali_t.scale, local_cali_t.offset, &count_time);
//    }
    //1.参数设置
    local_cali_t.scale[0] = 1.0f;
    local_cali_t.scale[1] = 1.0f;
    local_cali_t.scale[2] = 1.0f;

    //2.零漂计算
    //选择是否进行清零处理
  #ifdef RESET_OFFSET //½øÐÐÇåÁã´¦Àí
    flash_erase_address(FLASH_USER_ADDR,1);//²Á³öflash
  #endif
 
       //从flash中读取数据
    imu_start_dma_flag = 0;
  #ifdef RESET_OFFSET //再擦一遍
    flash_erase_address(FLASH_USER_ADDR,1);
    //获取矫正量
    Offset_Getdata();
    /* 矫正量放入flash中 */
    //仅写入角速度零漂数据  加速度和磁力计不用校正
    gyroOFFSet[0] = bmi088_off_data.gyro[0];
    gyroOFFSet[1] = bmi088_off_data.gyro[1];
    gyroOFFSet[2] = bmi088_off_data.gyro[2];
    //写入flash
	  WriteOffsetFromFlash(accOFFSet,gyroOFFSet,magOFFSet); 
  #endif
    GetOffsetFromFlash((float *) accOFFSet, (float *) gyroOFFSet, (float *) magOFFSet);
		
    bmi088_off_data.gyro[0] = (float)gyroOFFSet[0];
    bmi088_off_data.gyro[1] = (float)gyroOFFSet[1];
    bmi088_off_data.gyro[2] = (float)gyroOFFSet[2];


    //3.传递参数 
    gyro_offset[0] =  bmi088_off_data.gyro[0];
    gyro_offset[1] =  bmi088_off_data.gyro[1];
    gyro_offset[2] =  bmi088_off_data.gyro[2];
    local_cali_t.offset[0] = bmi088_off_data.gyro[0];
    local_cali_t.offset[1] = bmi088_off_data.gyro[1];
    local_cali_t.offset[2] = bmi088_off_data.gyro[2];
    //4.写入比例因子和零漂数据
    INS_set_cali_gyro(local_cali_t.scale, local_cali_t.offset);
    
    //进入循环前进行读数
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //rotate and zero drift
    bmi088Offset_Cal(&bmi088_real_data);

    //imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    //初始化四元素计算参数
    AHRS_init(INS_quat, INS_accel, INS_mag);
    //加速度计滤波
    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    //get the handle of task
    //获取当前任务的任务句柄，
    SensorTask_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    
    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    imu_start_dma_flag = 1;
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    while (1)
    {
        //wait spi DMA tansmit done
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }


        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);

        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
        
        //because no use ist8310 and save time, can no use ist8310readmag
        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1<< IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
            //ist8310_read_mag(ist8310_real_data.mag);
        }

        //rotate and zero drift
        //ÔÚ×ËÌ¬½âËãÇ°¶Ô½øÐÐÁãÆ¯Êý¾Ý´¦Àí
        bmi088Offset_Cal(&bmi088_real_data);
        INS_mag[0] = ist8310_real_data.mag[0];
        INS_mag[1] = ist8310_real_data.mag[1];
        INS_mag[2] = ist8310_real_data.mag[2];
        //imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

        //¼ÓËÙ¶È¼ÆµÍÍ¨ÂË²¨
        //accel low-pass filter
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];


        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat,   INS_angle + INS_YAW_ADDRESS_OFFSET,INS_angle + INS_ROLL_ADDRESS_OFFSET,INS_angle + INS_PITCH_ADDRESS_OFFSET);

    }
}


/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          Ðý×ªÍÓÂÝÒÇ,¼ÓËÙ¶È¼ÆºÍ´ÅÁ¦¼Æ,²¢¼ÆËãÁãÆ¯,ÒòÎªÉè±¸ÓÐ²»Í¬°²×°·½Ê½
  * @param[out]     gyro: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[out]     accel: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[out]     mag: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
  * @param[in]      bmi088: ÍÓÂÝÒÇºÍ¼ÓËÙ¶È¼ÆÊý¾Ý
  * @param[in]      ist8310: ´ÅÁ¦¼ÆÊý¾Ý
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}


static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_Calculate(&imu_temp_pid, temp, IMU_TEMP_CONTROL_SET);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        imu_pwm_set(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > IMU_TEMP_CONTROL_SET)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        imu_pwm_set(MPU6500_TEMP_PWM_MAX-1);
    }

		
}
 

/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
/**
  * @brief          ¼ÆËãÍÓÂÝÒÇÁãÆ¯
  * @param[out]     gyro_offset:¼ÆËãÁãÆ¯
  * @param[in]      gyro:½ÇËÙ¶ÈÊý¾Ý
  * @param[out]     offset_time_count: ×Ô¶¯¼Ó1
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          Ð£×¼ÍÓÂÝÒÇ
  * @param[out]     ÍÓÂÝÒÇµÄ±ÈÀýÒò×Ó£¬1.0fÎªÄ¬ÈÏÖµ£¬²»ÐÞ¸Ä
  * @param[out]     ÍÓÂÝÒÇµÄÁãÆ¯£¬²É¼¯ÍÓÂÝÒÇµÄ¾²Ö¹µÄÊä³ö×÷Îªoffset
  * @param[out]     ÍÓÂÝÒÇµÄÊ±¿Ì£¬Ã¿´ÎÔÚgyro_offsetµ÷ÓÃ»á¼Ó1,
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          Ð£×¼ÍÓÂÝÒÇÉèÖÃ£¬½«´Óflash»òÕßÆäËûµØ·½´«ÈëÐ£×¼Öµ
  * @param[in]      ÍÓÂÝÒÇµÄ±ÈÀýÒò×Ó£¬1.0fÎªÄ¬ÈÏÖµ£¬²»ÐÞ¸Ä
  * @param[in]      ÍÓÂÝÒÇµÄÁãÆ¯
  * @retval         none
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */
/**
  * @brief          »ñÈ¡ËÄÔªÊý
  * @param[in]      none
  * @retval         INS_quatµÄÖ¸Õë
  */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}
/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          »ñÈ¡Å·À­½Ç, 0:yaw, 1:pitch, 2:roll µ¥Î» rad
  * @param[in]      none
  * @retval         INS_angleµÄÖ¸Õë
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          »ñÈ¡½ÇËÙ¶È,0:xÖá, 1:yÖá, 2:rollÖá µ¥Î» rad/s
  * @param[in]      none
  * @retval         INS_gyroµÄÖ¸Õë
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}
/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_accel
  */
/**
  * @brief          »ñÈ¡¼ÓËÙ¶È,0:xÖá, 1:yÖá, 2:rollÖá µ¥Î» m/s2
  * @param[in]      none
  * @retval         INS_accelµÄÖ¸Õë
  */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}
/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
/**
  * @brief          »ñÈ¡¼ÓËÙ¶È,0:xÖá, 1:yÖá, 2:rollÖá µ¥Î» ut
  * @param[in]      none
  * @retval         INS_magµÄÖ¸Õë
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        //detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        //detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == IST8310_DRDY_Pin)
    {
        //detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {

        //wake up the task
        //»½ÐÑÈÎÎñ
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(SensorTask_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }
		
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ¸ù¾Ýimu_update_flagµÄÖµ¿ªÆôSPI DMA
  * @param[in]      temp:bmi088µÄÎÂ¶È
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{

        //¿ªÆôÍÓÂÝÒÇµÄDMA´«Êä
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
        //¿ªÆô¼ÓËÙ¶È¼ÆµÄDMA´«Êä
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
        


        
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}

void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //ÍÓÂÝÒÇ¶ÁÈ¡Íê±Ï
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //¼ÓËÙ¶È¼Æ¶ÁÈ¡Íê±Ï
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //ÎÂ¶È¶ÁÈ¡Íê±Ï
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        
        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

    

/**
  * @brief          get imu control temperature, unit ¡æ
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief          »ñÈ¡imu¿ØÖÆÎÂ¶È, µ¥Î»¡æ
  * @param[in]      none
  * @retval         imu¿ØÖÆÎÂ¶È
  */
int8_t get_control_temperature(void)
{
    return head_cali.temperature;
}

void updata_imu_temperature(void)
{
    head_cali.temperature = (int8_t)(get_temprate()) + 10;
    if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }   
}

static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;//ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}



void bmi088Offset_Cal(bmi088_real_data_t *bmiOffSetCal)
{
    bmiOffSetCal->gyro[0] -= bmi088_off_data.gyro[0];
    bmiOffSetCal->gyro[1] -= bmi088_off_data.gyro[1];
    bmiOffSetCal->gyro[2] -= bmi088_off_data.gyro[2];
    

    for (uint8_t i = 0; i < 3; i++)
    {
        INS_gyro[i] = bmiOffSetCal->gyro[0] * gyro_scale_factor[i][0] + bmiOffSetCal->gyro[1] * gyro_scale_factor[i][1] + bmiOffSetCal->gyro[2] * gyro_scale_factor[i][2] ;
        INS_accel[i] = bmiOffSetCal->accel[0] * accel_scale_factor[i][0] + bmiOffSetCal->accel[1] * accel_scale_factor[i][1] + bmiOffSetCal->accel[2] * accel_scale_factor[i][2] ;
    }

}

static void Offset_Getdata(void)
{
	uint16_t t = ZERODRIFT_TIMES; //Ä¿±ê¼ÆÊýÖµ
    
    uint16_t cnt = 0; //¼ÆÊýÆ÷
	
    
	while(cnt < ZERODRIFT_TIMES)
	{
		BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
        //¼ÌÐøÎÂ¶È¿ØÖÆ
        imu_temp_control(bmi088_real_data.temp);
        
        //ÔÚÒ»¶¨ÎÂ¶ÈãÐÖµÄÚ½øÐÐÁãÆ¯Êý¾Ý¼ÇÂ¼
        if(bmi088_real_data.temp > IMU_TEMP_CONTROL_SET - 0.3f && bmi088_real_data.temp < IMU_TEMP_CONTROL_SET + 0.3f)
        {
            bmi088_off_data.accel[0] += bmi088_real_data.accel[0];
            bmi088_off_data.accel[1] += bmi088_real_data.accel[1];
            bmi088_off_data.accel[2] += bmi088_real_data.accel[2];
                
            bmi088_off_data.gyro[0] += bmi088_real_data.gyro[0];
            bmi088_off_data.gyro[1] += bmi088_real_data.gyro[1];
            bmi088_off_data.gyro[2] += bmi088_real_data.gyro[2];
            cnt ++;
        }		
		osDelay(1);
	}
	
	bmi088_off_data.accel[0] /=ZERODRIFT_TIMES;
	bmi088_off_data.accel[1] /=ZERODRIFT_TIMES;
	bmi088_off_data.accel[2] /=ZERODRIFT_TIMES;
	bmi088_off_data.gyro[0] /=ZERODRIFT_TIMES;
	bmi088_off_data.gyro[1] /=ZERODRIFT_TIMES;
	bmi088_off_data.gyro[2] /=ZERODRIFT_TIMES;
   
	bmi088_real_data.accel[0] = 0;
	bmi088_real_data.accel[1] = 0;
    bmi088_real_data.accel[2] = 0;
	bmi088_real_data.gyro[0] = 0;
	bmi088_real_data.gyro[1] = 0;
	bmi088_real_data.gyro[2] = 0;
}

/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ÍùflashÐ´ÈëÐ£×¼Êý¾Ý
  * @param[in]      none
  * @retval         none
  */

/**
 * @Description: ´ÓflashµÃµ½ÍÓÂÝÒÇÊýÖµ
 * @Auther: zd
 * @Date: 2021-01-16 13:28:16
 * @param {int32_t} accOFFSet 	¼ÓËÙ¶ÈÆ«ÒÆ
 * @param {int32_t} gyroOFFSet	½ÇËÙ¶ÈÆ«ÒÆ
 * @param {int32_t} magOFFset	´ÅÁ¦¼ÆÆ«ÒÆ
 */
void GetOffsetFromFlash(float*  accOFFSet,float * gyroOFFSet,float * magOFFset)
{
	uint8_t i;
	float getBuffer[FLASH_LENGTH];
	//´ÓflashÖÐ¶ÁÈ¡9¸ö32Î»
	flash_read(FLASH_USER_ADDR,(uint32_t *)getBuffer,sizeof(getBuffer)/4U);
	for (i = 0; i < FLASH_LENGTH; i ++)
	{
		if (i < FLASH_LENGTH/3)
		{
			*(accOFFSet + i) = getBuffer[i];
		}
		else if (i < 2*FLASH_LENGTH/3)
		{
			*(gyroOFFSet + i -3) = getBuffer[i];
		}
		else
		{
			*(magOFFset	+ i - 6) = getBuffer[i];
		}
	} 
}

/**
 * @Description: ½«Ð£×¼ÊýÖµÐ´ÈëFlash
 * @Auther: zd
 * @Date: 2021-01-16 13:28:16
 * @param {int32_t} accOFFSet 	¼ÓËÙ¶ÈÆ«ÒÆ
 * @param {int32_t} gyroOFFSet	½ÇËÙ¶ÈÆ«ÒÆ
 * @param {int32_t} magOFFset	´ÅÁ¦¼ÆÆ«ÒÆ
 */
void WriteOffsetFromFlash(float accOFFSet[3],float gyroOFFSet[3],float magOFFset[3])
{
float writeBuffer[FLASH_LENGTH];
	uint8_t i;
	for (i = 0; i < FLASH_LENGTH; i ++)
	{
		if (i < FLASH_LENGTH/3)
		{
			writeBuffer[i] = accOFFSet[i];
		}
		else if (i < 2*FLASH_LENGTH/3)
		{
			writeBuffer[i] = gyroOFFSet[i - 3];
		}
		else
		{
			writeBuffer[i] = magOFFset[i - 6];
		}
	}
	//Ð´ÈëflashÇ°ÒªÏÈÇå³ýÊý¾Ý
	flash_erase_address(FLASH_USER_ADDR,1);
	/* flashÐ´ÈëºÃÏñÊÇ32Î»µÄ ËùÒÔÐèÒª³ýËÄ */
	flash_write_single_address(FLASH_USER_ADDR,(uint32_t *)writeBuffer,sizeof(writeBuffer)/4U);
}




