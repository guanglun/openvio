#ifndef __OPENVIO_TASK_H__
#define __OPENVIO_TASK_H__

#include "usbd_def.h"

void StartOpenvioTask(void const * argument);
uint8_t camera_ctrl(USBD_SetupReqTypedef *req,uint8_t *s_data);

#endif


