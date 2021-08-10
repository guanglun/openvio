#ifndef __UART_PARSE_H__
#define __UART_PARSE_H__

#include "main.h"
#include "protocol.h"

void uart_receive_struct_init(void);
void receive_uart_data(uint8_t *receive_buffer,uint16_t receive_len);
void uart_parse_loop(void);

#endif

