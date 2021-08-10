/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2019 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2019 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 */
#include <stddef.h>
#include "cambus.h"

//#include "FreeRTOS.h"
//#include "task.h"
#include "main.h"
//#include "cmsis_os.h"

#define I2C_TIMEOUT     (1000)

extern I2C_HandleTypeDef hi2c1;

int cambus_scan(void)
{
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
        __disable_irq();
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 10, I2C_TIMEOUT) == HAL_OK) {
            __enable_irq();
            return (addr << 1);
        }
        __enable_irq();
    }

    return 0;
}

int cambus_readb(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data)
{
    int ret = 0;

    __disable_irq();
    if((HAL_I2C_Master_Transmit(&hi2c1, slv_addr, &reg_addr, 1, I2C_TIMEOUT) != HAL_OK)
    || (HAL_I2C_Master_Receive(&hi2c1, slv_addr, reg_data, 1, I2C_TIMEOUT) != HAL_OK)) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

int cambus_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data)
{
    int ret=0;
    uint8_t buf[] = {reg_addr, reg_data};

    __disable_irq();
    if(HAL_I2C_Master_Transmit(&hi2c1, slv_addr, buf, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

int cambus_readb2(uint8_t slv_addr, uint16_t reg_addr, uint8_t *reg_data)
{
    int ret=0;
    __disable_irq();
    if (HAL_I2C_Mem_Read(&hi2c1, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_16BIT, reg_data, 1, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

int cambus_writeb2(uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data)
{
    int ret=0;
    __disable_irq();
    if (HAL_I2C_Mem_Write(&hi2c1, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_16BIT, &reg_data, 1, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

int cambus_readw(uint8_t slv_addr, uint8_t reg_addr, uint16_t *reg_data)
{
    int ret=0;
    __disable_irq();
    if (HAL_I2C_Mem_Read(&hi2c1, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_8BIT, (uint8_t*) reg_data, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    *reg_data = (*reg_data >> 8) | (*reg_data << 8);
    return ret;
}

int cambus_writew(uint8_t slv_addr, uint8_t reg_addr, uint16_t reg_data)
{
    int ret=0;
    reg_data = (reg_data >> 8) | (reg_data << 8);
    __disable_irq();
    if (HAL_I2C_Mem_Write(&hi2c1, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_8BIT, (uint8_t*) &reg_data, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

int cambus_readw2(uint8_t slv_addr, uint16_t reg_addr, uint16_t *reg_data)
{
    int ret=0;
    __disable_irq();
    if (HAL_I2C_Mem_Read(&hi2c1, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_16BIT, (uint8_t*) reg_data, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    *reg_data = (*reg_data >> 8) | (*reg_data << 8);
    return ret;
}

int cambus_writew2(uint8_t slv_addr, uint16_t reg_addr, uint16_t reg_data)
{
    int ret=0;
    reg_data = (reg_data >> 8) | (reg_data << 8);
    __disable_irq();
    if (HAL_I2C_Mem_Write(&hi2c1, slv_addr, reg_addr,
                I2C_MEMADD_SIZE_16BIT, (uint8_t*) &reg_data, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}
