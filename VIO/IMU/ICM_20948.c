#include "ICM_20948.h"

#include "ICM_20948_REGISTERS.h"
#include "AK09916_REGISTERS.h"

#include "spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

#define ICM20948_ENABLE() HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_RESET)
#define ICM20948_DISABLE() HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_SET)

#define ICM20948_TIMEOUT_VALUE 0xFFFF

ICM_20948_Device_t _device;

ICM_20948_Status_e my_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{

    uint8_t cmd = ((reg & 0x7F) | 0x00);

    ICM20948_ENABLE();

    if (HAL_SPI_Transmit(&hspi2, &cmd, 1, ICM20948_TIMEOUT_VALUE) != HAL_OK)
    {
        return ICM_20948_Stat_Err;
    }

    if (HAL_SPI_Transmit(&hspi2, data, len, ICM20948_TIMEOUT_VALUE) != HAL_OK)
    {
        return ICM_20948_Stat_Err;
    }

    ICM20948_DISABLE();

    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e my_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{

    uint8_t cmd = (((reg & 0x7F) | 0x80));

    ICM20948_ENABLE();

    if (HAL_SPI_Transmit(&hspi2, &cmd, 1, ICM20948_TIMEOUT_VALUE) != HAL_OK)
    {
        return ICM_20948_Stat_Err;
    }

    if (HAL_SPI_Receive(&hspi2, buff, len, ICM20948_TIMEOUT_VALUE) != HAL_OK)
    {
        return ICM_20948_Stat_Err;
    }

    ICM20948_DISABLE();
    return ICM_20948_Stat_Ok;
}
ICM_20948_Status_e status;  
const ICM_20948_Serif_t mySerif = {
    my_write_spi, // write
    my_read_spi,  // read
    NULL,         // this pointer is passed into your functions when they are called.
};

ICM_20948_Status_e i2cMasterPassthrough(bool passthrough)
{
    status = ICM_20948_i2c_master_passthrough(&_device, passthrough);
    return status;
}
ICM_20948_Status_e i2cMasterEnable(bool enable)
{
    status = ICM_20948_i2c_master_enable(&_device, enable);
    return status;
}


ICM_20948_Status_e i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data)
{
    status = ICM_20948_i2c_master_single_w(&_device, addr, reg, &data);
    return status;
}
uint8_t i2cMasterSingleR(uint8_t addr, uint8_t reg)
{
    uint8_t data;
    status = ICM_20948_i2c_master_single_r(&_device, addr, reg, &data);
    return data;
}

uint8_t readMag(AK09916_Reg_Addr_e reg)
{
    uint8_t data = i2cMasterSingleR(MAG_AK09916_I2C_ADDR, reg);
    return data;
}

ICM_20948_Status_e writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
    status = i2cMasterSingleW(MAG_AK09916_I2C_ADDR, reg, *pdata);
    return status;
}

ICM_20948_Status_e magWhoIAm(void)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    uint8_t whoiam1, whoiam2;
    whoiam1 = readMag(AK09916_REG_WIA1);
    whoiam2 = readMag(AK09916_REG_WIA2);
    status = retval;
    if (retval != ICM_20948_Stat_Ok)
    {
        return retval;
    }

    if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
    {
        retval = ICM_20948_Stat_Ok;
        status = retval;
        return status;
    }
    retval = ICM_20948_Stat_WrongID;
    status = retval;
    return status;
}
ICM_20948_Status_e i2cMasterReset()
{
    status = ICM_20948_i2c_master_reset(&_device);
    return status;
}

ICM_20948_Status_e i2cMasterConfigureSlave(uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap)
{
    status = ICM_20948_i2c_master_configure_slave(&_device, slave, addr, reg, len, Rw, enable, data_only, grp, swap);
    return status;
}

ICM_20948_Status_e startupMagnetometer(void)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    i2cMasterPassthrough(false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
    i2cMasterEnable(true);

    //After a ICM reset the Mag sensor may stop responding over the I2C master
    //Reset the Master I2C until it responds
    uint8_t tries = 0;
    uint8_t maxTries = 5;
    while (tries < maxTries)
    {
        //See if we can read the WhoIAm register correctly
        retval = magWhoIAm();
        if (retval == ICM_20948_Stat_Ok)
            break; //WIA matched!

        i2cMasterReset(); //Otherwise, reset the master I2C and try again
        tries++;
    }

    if (tries == maxTries)
    {
        status = ICM_20948_Stat_WrongID;
        return status;
    }

    //Serial.printf("Mag connected tries: %d\n", tries);

    //Set up magnetometer
    AK09916_CNTL2_Reg_t reg;
    reg.MODE = AK09916_mode_cont_100hz;
    retval = writeMag(AK09916_REG_CNTL2, (uint8_t *)&reg);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    retval = i2cMasterConfigureSlave(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    return status;
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  printf("RAW. Acc [ %d,%d,%d ], Gyr [ %d,%d,%d ], Mag [ %d,%d,%d ], Tmp [ %d ]\r\n", \
  agmt.acc.axes.x,agmt.acc.axes.y,agmt.acc.axes.z, \
  agmt.gyr.axes.x,agmt.gyr.axes.y,agmt.gyr.axes.z, \
  agmt.mag.axes.x,agmt.mag.axes.y,agmt.mag.axes.z, \
  agmt.tmp.val);
}

void icm_init_new(void)
{
    ICM_20948_link_serif(&_device, &mySerif);
	if(startupMagnetometer() != ICM_20948_Stat_Ok)
	{
		printf("startupMagnetometer Error\r\n");
	}
}

void icm_get_agmt(void)
{
  ICM_20948_AGMT_t agmt = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
  if(ICM_20948_get_agmt( &_device, &agmt ) == ICM_20948_Stat_Ok){
      printRawAGMT( agmt );
  }else{
      printf("Uh oh\r\n");
  }
}

void icm_get_agmt_buff(uint8_t *buff)
{
    ICM_20948_get_agmt_buff(&_device, buff);
}
