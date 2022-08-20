#ifndef _SENSORTASK_H
#define _SENSORTASK_H

#include "main.h"

#include "cmsis_os.h"


#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "adc.h"
#include "ahrs.h"
#include "struct_typedef.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS        2


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//flashʹ�õĵ�ַ�Ͷ�ȡ�����ݳ���
#define FLASH_USER_ADDR ADDR_FLASH_SECTOR_9 
#define FLASH_LENGTH    9

//����flash����
//#define RESET_OFFSET 

//ist83100ԭʼ�����ڻ�����buf��λ��
#define IST8310_RX_BUF_DATA_OFFSET 16

//IMU�¶ȿ�����ʽ
//IMU_TEMP_CONTROL_STYLE = 1 Ϊ�̶�ֵ���� 
//IMU_TEMP_CONTROL_STYLE = 2 ΪADC��ȡ�����¶��ڰ����¶�������10���϶ȿ���
#define IMU_TEMP_CONTROL_STYLE 1

#if IMU_TEMP_CONTROL_STYLE == 1
#define IMU_TEMP_CONTROL_SET 50.0f
#endif


#define TEMPERATURE_PID_KP 1600.0f //�¶ȿ���PID��kp
#define TEMPERATURE_PID_KI 0.1f    //�¶ȿ���PID��ki
#define TEMPERATURE_PID_KD 0.0f    //�¶ȿ���PID��kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //�¶ȿ���PID��max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //�¶ȿ���PID��max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM���Ϊ MPU6500_TEMP_PWM_MAX - 1


#define INS_TASK_INIT_TIME 7 //����ʼ���� delay һ��ʱ��
#define ZERODRIFT_TIMES 10000 //��Ư����ʱ�� ���ں�������ֱ�Ӵ���flash �������ֵ���Դ�һ��

#define INS_YAW_ADDRESS_OFFSET    2
#define INS_PITCH_ADDRESS_OFFSET  0
#define INS_ROLL_ADDRESS_OFFSET   1

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

typedef enum
{
    CALI_HEAD = 0,
    CALI_GIMBAL = 1,
    CALI_GYRO = 2,
    CALI_ACC = 3,
    CALI_MAG = 4,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;

typedef __packed struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);   //cali function
} cali_sensor_t;

//IMU �¶ȿ���
#define GYRO_CONST_MAX_TEMP     50.0f 
#define ADC_CHANNEL_TEMPSENSOR  ((uint32_t)ADC_CHANNEL_16)

//header device
typedef __packed struct
{
    uint8_t self_id;            // the "SELF_ID"
    uint16_t firmware_version;  // set to the "FIRMWARE_VERSION"
    //'temperature' and 'latitude' should not be in the head_cali, because don't want to create a new sensor
    //'temperature' and 'latitude'��Ӧ����head_cali,��Ϊ���봴��һ���µ��豸�ͷ�����
    int8_t temperature;         // imu control temperature
    fp32 latitude;              // latitude
} head_cali_t;

typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;
#define FLASH_WRITE_BUF_LENGHT  (sizeof(imu_cali_t) * 3)


/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void INS_task(void const *pvParameters);

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          У׼������
  * @param[out]     �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[out]     �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[out]     �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  * @retval         none
  */
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          У׼���������ã�����flash���������ط�����У׼ֵ
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư
  * @retval         none
  */
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */
/**
  * @brief          ��ȡ��Ԫ��
  * @param[in]      none
  * @retval         INS_quat��ָ��
  */
extern const fp32 *get_INS_quat_point(void);


/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          ��ȡŷ����, 0:yaw, 1:pitch, 2:roll ��λ rad
  * @param[in]      none
  * @retval         INS_angle��ָ��
  */
extern const fp32 *get_INS_angle_point(void);


/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ rad/s
  * @param[in]      none
  * @retval         INS_gyro��ָ��
  */
extern const fp32 *get_gyro_data_point(void);


/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ m/s2
  * @param[in]      none
  * @retval         INS_gyro��ָ��
  */
extern const fp32 *get_accel_data_point(void);

/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ ut
  * @param[in]      none
  * @retval         INS_mag��ָ��
  */
extern const fp32 *get_mag_data_point(void);


/**
  * @brief          get imu control temperature, unit ��
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief          ��ȡimu�����¶�, ��λ��
  * @param[in]      none
  * @retval         imu�����¶�
  */
extern int8_t get_control_temperature(void);


/**
  * @brief          ADC��ȡ�¶�, ��λ��
  * @param[in]      none
  * @retval         ADC��ȡʵ���¶��¶�
  */


extern void bmi088Offset_Cal(bmi088_real_data_t *bmiOffSetCal);

 extern bmi088_real_data_t bmi088_real_data;
#endif
