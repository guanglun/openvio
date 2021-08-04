#ifndef __ICM20948_H__
#define __ICM20948_H__

#include "main.h"

#define USER_BANK_SEL	(0x7F)
#define USER_BANK_0		(0x00)
#define USER_BANK_1		(0x10)
#define USER_BANK_2		(0x20)
#define USER_BANK_3		(0x30)

#define PWR_MGMT_1 		(0x06)
#define PWR_MGMT_2		(0x07)
#define GYRO_CONFIG_1	(0x01)


#define CLK_BEST_AVAIL	(0x01)
#define GYRO_RATE_250	(0x00)
#define GYRO_RATE_500	(0x01)
#define GYRO_RATE_1000	(0x02)
#define GYRO_RATE_2000	(0x03)
#define GYRO_LPF_17HZ 	(0x29)

int icm20948_init(void);
void icm20948_read(uint8_t *buf);
void icm20948_transmit(void);

#endif

