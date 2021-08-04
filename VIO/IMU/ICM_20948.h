/*

A C++ interface to the ICM-20948

*/

#ifndef _ICM_20948_H_
#define _ICM_20948_H_

#include "ICM_20948_C.h" // The C backbone
#include "AK09916_REGISTERS.h"

void icm_init_new(void);
void icm_get_agmt(void);
void icm_get_agmt_buff(uint8_t *buff);

#endif /* _ICM_20948_H_ */