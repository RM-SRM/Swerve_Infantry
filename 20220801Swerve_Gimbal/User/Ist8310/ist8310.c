#include "ist8310.h"
#include "spi.h"
#include "gpio.h"
#include "mpu6500.h"
#include "cmsis_os.h"

#define MAG_SEN 0.3f 

#define IST8310_WHO_AM_I 0x00       //ist8310 who am I µÿ÷∑
#define IST8310_WHO_AM_I_VALUE 0x10 

#define IST8310_WRITE_REG_NUM 4 //IST8310

ist8310_real_data_t ist8310_data;
uint8_t res = 0,i;

void ist8310_com_init(void)
{
}

void ist8310_auto_com_by_mpu6500(void)
{
    uint8_t readBuf[3] = {IST8310_IIC_ADDRESS | IST8310_IIC_READ_MSB, 0x02, 0x8d};
    MPU6500_Write_Regs(MPU6500_I2C_SLV0_ADDR, readBuf, 3);
}

uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    static const uint16_t IIC_time = 2;
    uint8_t readBuf[3] = {IST8310_IIC_ADDRESS | IST8310_IIC_READ_MSB, reg, 0x81};
    MPU6500_Write_Regs(MPU6500_I2C_SLV0_ADDR, readBuf, 3);
	osDelay(2);														//here!!
    return MPU6500_Read_Reg(MPU6500_EXT_SENS_DATA_00);
}

void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    uint8_t writeBuf[4] = {IST8310_IIC_ADDRESS, reg, data, 0x80};

    MPU6500_Write_Regs(MPU6500_I2C_SLV4_ADDR, writeBuf, 4);
}

void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    while (len)
    {
        (*buf) = ist8310_IIC_read_single_reg(reg);
        reg++;
        buf++;
        len--;
    }
}

void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    static const uint16_t IIC_time = 2000;
    while (len)
    {
        ist8310_IIC_write_single_reg(reg, (*data));
        reg++;
        data++;
        len--;
        ist8310_delay_us(IIC_time);
    }
}

void ist8310_delay_ms(uint16_t ms)
{
    //delay_ms(ms);
	osDelay(ms);
}
void ist8310_delay_us(uint16_t us)
{
    //delay_us(us);
}

void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
}

extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
}

static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] =
    {
        {0x0B, 0x08, 0x01},
        {0x41, 0x09, 0x02},
        {0x42, 0xC0, 0x03},
        {0x0A, 0x0B, 0x03}};
	
uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;

    uint8_t writeNum = 0;

    ist8310_com_init();

    ist8310_RST_L();
    ist8310_delay_ms(sleepTime);
    ist8310_delay_ms(sleepTime);
    ist8310_RST_H();

    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }

    //set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
		for (i=0; i< wait_time; i++)
        ist8310_delay_us(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
		for (i=0; i< wait_time; i++)
        ist8310_delay_us(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    ist8310_auto_com_by_mpu6500();

    return IST8310_NO_ERROR;
}

void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{

    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

void ist8310_read_mag(ist8310_real_data_t *ist8310_real_data)
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    ist8310_IIC_read_muli_reg(0x02, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
}
