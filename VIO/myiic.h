#ifndef __MYIIC_H__
#define __MYIIC_H__

#include "stm32h7xx.h"

/*设定的MPU6050 IIC设备地址*/
#define MPU6050_ADDR              0xD0

#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

/*I2C引脚*/
#define MPU6050_I2C_SCL_PIN                  GPIO_PIN_11                 
#define MPU6050_I2C_SCL_GPIO_PORT            GPIOD                       
#define MPU6050_I2C_SCL_GPIO_CLK_ENABLE()    __GPIOD_CLK_ENABLE()

#define MPU6050_I2C_SDA_PIN                  GPIO_PIN_10                 
#define MPU6050_I2C_SDA_GPIO_PORT            GPIOD                    
#define MPU6050_I2C_SDA_GPIO_CLK_ENABLE()    __GPIOD_CLK_ENABLE()

//软件IIC使用的宏
#define I2C_SCL_1()  HAL_GPIO_WritePin(MPU6050_I2C_SCL_GPIO_PORT, MPU6050_I2C_SCL_PIN,GPIO_PIN_SET)		/* SCL = 1 */
#define I2C_SCL_0()  HAL_GPIO_WritePin(MPU6050_I2C_SCL_GPIO_PORT, MPU6050_I2C_SCL_PIN,GPIO_PIN_RESET)		/* SCL = 0 */

#define I2C_SDA_1()  HAL_GPIO_WritePin(MPU6050_I2C_SDA_GPIO_PORT, MPU6050_I2C_SDA_PIN,GPIO_PIN_SET)		/* SDA = 1 */
#define I2C_SDA_0()  HAL_GPIO_WritePin(MPU6050_I2C_SDA_GPIO_PORT, MPU6050_I2C_SDA_PIN,GPIO_PIN_RESET)		/* SDA = 0 */

#define I2C_SDA_READ()  HAL_GPIO_ReadPin(MPU6050_I2C_SDA_GPIO_PORT, MPU6050_I2C_SDA_PIN)	/* 读SDA口线状态 */


//函数接口
void IMU_I2C_Init(void);
uint32_t I2C_WriteBytes(uint8_t ClientAddr,uint8_t* pBuffer,  uint8_t NumByteToWrite);
uint32_t I2C_ReadBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint16_t NumByteToRead);
uint32_t Sensor_write(uint8_t reg_add,uint8_t reg_dat);
uint32_t Sensor_Read(uint8_t reg_add,unsigned char* Read,uint8_t num);
#endif /* __BSP_I2C */
