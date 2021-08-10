#ifndef __USB_PARSE_H__
#define __USB_PARSE_H__

#include "main.h"
#include "protocol.h"

void usb_receive_struct_init(void);
void receive_usb_data(uint8_t *receive_buffer,uint16_t receive_len);
void usb_parse_loop(void);

#endif

