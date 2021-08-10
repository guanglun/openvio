#include "usb_parse.h"
#include "usb_device.h"
#include "iap.h"
#include "protocol.h"
#include "config.h"

#define VERSION_1 0
#define VERSION_2 0
#define VERSION_3 1

extern USBD_HandleTypeDef hUsbDeviceHS;

PARSE_STRUCT parse_usb;
uint8_t is_usb_recv = 0;
void send_usb_data(uint8_t *send_buffer,uint16_t send_len);

static void usb_printf_frame(FRAME_STRUCT *frame_s)
{
    printf("Version:%02X ",frame_s->Version);
    printf("FrameDataLen:%02X ",frame_s->FrameDataLen);
    printf("SourceID:%02X ",frame_s->SourceID);
    printf("TargetID:%02X ",frame_s->TargetID);
    printf("Cmd:%02X ",frame_s->Cmd);
    printf("DataIndex:%02X ",frame_s->DataIndex);
    //printf("Data:");
    //printf_byte(frame_s->frame_data,frame_s->FrameDataLen);
    printf("\r\n");
}

void recv_usb_farme(void *arg)
{
    is_usb_recv = 1;
}

uint8_t recv_usb_buffer[1024];
uint16_t recv_usb_len = 0;

void usb_parse_loop(void)
{
    if(is_usb_recv != 0)
    {
        usb_printf_frame(&parse_usb.frame_s);
        if(parse_usb.frame_s.TargetID == LOCAL_ID)
        {
            parse_iap_frame(&parse_usb);
             
        }else{

        }   
        is_usb_recv = 0;     
    }
}

void usb_receive_struct_init(void)
{
	parse_struct_init(&parse_usb);
	parse_set_rec_callback(&parse_usb,recv_usb_farme);
    parse_set_send_fun(&parse_usb,send_usb_data);
}

void receive_usb_data(uint8_t *receive_buffer,uint16_t receive_len)
{
	parse_data(&parse_usb,receive_buffer,receive_len);
}

void send_usb_data(uint8_t *send_buffer,uint16_t send_len)
{
	CDC_Transmit_HS(send_buffer,send_len);
}

// int return_status(uint8_t status, uint8_t power)
// {
//     unsigned char send_buf[4];

//     send_buf[0] = status;
//     send_buf[1] = power;

//     FRAME_STRUCT frame_s_tmp;
//     frame_s_tmp.Version = PROTOCOL_VERSION;
//     frame_s_tmp.FrameDataLen = 0;
//     frame_s_tmp.SourceID = LOCAL_ID;
//     frame_s_tmp.TargetID = TARGET_ID;
//     frame_s_tmp.Cmd = 0x65;
//     frame_s_tmp.DataIndex = 0x00;
//     frame_s_tmp.frame_data = send_buf;
//     frame_s_tmp.send_frame_fun = send_usb_data;

//     return creat_send_cmd(&parse_usb, &frame_s_tmp);
// }

// int send_bootloader_version(void)
// {
//     unsigned char send_buf[3];

//     send_buf[0] = VERSION_1;
//     send_buf[1] = VERSION_2;
//     send_buf[2] = VERSION_3;

//     FRAME_STRUCT frame_s_tmp;
//     frame_s_tmp.Version = PROTOCOL_VERSION;
//     frame_s_tmp.FrameDataLen = 3;
//     frame_s_tmp.SourceID = LOCAL_ID;
//     frame_s_tmp.TargetID = TARGET_ID;
//     frame_s_tmp.Cmd = CMD_REPORT_BOOTLOADER_VERSION;
//     frame_s_tmp.DataIndex = 0x00;
//     frame_s_tmp.frame_data = send_buf;
//     frame_s_tmp.send_frame_fun = send_usb_data;

//     return creat_send_cmd(&parse_usb, &frame_s_tmp);
// }

