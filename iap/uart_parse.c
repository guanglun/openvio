#include "uart_parse.h"
#include "usart.h"
#include "iap.h"
#include "protocol.h"
#include "config.h"

#define VERSION_1 0
#define VERSION_2 0
#define VERSION_3 1

PARSE_STRUCT parse_uart;
uint8_t is_recv = 0;
void send_uart_data(uint8_t *send_buffer,uint16_t send_len);

void printf_frame(FRAME_STRUCT *frame_s)
{
    // Log("Version:%02X ",frame_s->Version);
    // Log("FrameDataLen:%02X ",frame_s->FrameDataLen);
    // Log("SourceID:%02X ",frame_s->SourceID);
    // Log("TargetID:%02X ",frame_s->TargetID);
    // Log("Cmd:%02X ",frame_s->Cmd);
    // Log("DataIndex:%02X ",frame_s->DataIndex);
    // Log("Data:");
    // printf_byte(frame_s->frame_data,frame_s->FrameDataLen);
    // Log("\r\n");
}

void recv_uart_farme(void *arg)
{
    is_recv = 1;
}

uint8_t recv_buffer[1024];
uint16_t recv_len = 0;

void uart_parse_loop(void)
{

    if(is_recv != 0)
    {
        
        if(parse_uart.frame_s.TargetID == LOCAL_ID)
        {
            parse_iap_frame(&parse_uart);
             
        }else{

        }   
        is_recv = 0;     
    }
}

void uart_receive_struct_init(void)
{
	parse_struct_init(&parse_uart);
	parse_set_rec_callback(&parse_uart,recv_uart_farme);
    parse_set_send_fun(&parse_uart,send_uart_data);
}

void receive_uart_data(uint8_t *receive_buffer,uint16_t receive_len)
{
	parse_data(&parse_uart,receive_buffer,receive_len);
}

void send_uart_data(uint8_t *send_buffer,uint16_t send_len)
{
	uart_transmit_buffer(send_buffer,send_len);
}

int return_status(uint8_t status, uint8_t power)
{
    unsigned char send_buf[4];

    send_buf[0] = status;
    send_buf[1] = power;

    FRAME_STRUCT frame_s_tmp;
    frame_s_tmp.Version = PROTOCOL_VERSION;
    frame_s_tmp.FrameDataLen = 0;
    frame_s_tmp.SourceID = LOCAL_ID;
    frame_s_tmp.TargetID = TARGET_ID;
    frame_s_tmp.Cmd = 0x65;
    frame_s_tmp.DataIndex = 0x00;
    frame_s_tmp.frame_data = send_buf;
    frame_s_tmp.send_frame_fun = send_uart_data;

    return creat_send_cmd(&parse_uart, &frame_s_tmp);
}

int send_bootloader_version(void)
{
    unsigned char send_buf[3];

    send_buf[0] = VERSION_1;
    send_buf[1] = VERSION_2;
    send_buf[2] = VERSION_3;

    FRAME_STRUCT frame_s_tmp;
    frame_s_tmp.Version = PROTOCOL_VERSION;
    frame_s_tmp.FrameDataLen = 3;
    frame_s_tmp.SourceID = LOCAL_ID;
    frame_s_tmp.TargetID = TARGET_ID;
    frame_s_tmp.Cmd = CMD_REPORT_BOOTLOADER_VERSION;
    frame_s_tmp.DataIndex = 0x00;
    frame_s_tmp.frame_data = send_buf;
    frame_s_tmp.send_frame_fun = send_uart_data;

    return creat_send_cmd(&parse_uart, &frame_s_tmp);
}

