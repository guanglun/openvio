#ifndef __OPENVIO_TASK_H__
#define __OPENVIO_TASK_H__

#include "usbd_def.h"

void StartOpenvioTask(void const * argument);
int camera_ctrl(uint8_t cmd, uint8_t* pbuf);
int camera_recv(uint8_t cmd, uint8_t* pbuf, uint16_t length);

#endif


