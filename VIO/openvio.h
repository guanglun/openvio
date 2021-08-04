#ifndef __OPENVIO_H__
#define __OPENVIO_H__

#include "openvio_def.h"

extern struct OPENVIO_STATUS vio_status;

void openvio_status_init(struct OPENVIO_STATUS *status);
uint8_t openvio_usb_send(enum SENSOR_USB usb,uint8_t* Buf, uint32_t Len);
uint8_t CAM_Transmit_HS(uint8_t* Buf, uint32_t Len);
uint8_t get_usb_tx_state(void);

#endif
