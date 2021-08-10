#ifndef __IAP_H__
#define __IAP_H__

#include "main.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "stdio.h"
#include "config.h"

#include "protocol.h"

#define CMD_GET_VERSION	0x11

#define CMD_REPORT_BOOTLOADER_VERSION	0x21
#define CMD_REPORT_APP_VERSION	        0x22

#define CMD_IAP_BEGIN   0xA1		//IAP开启指令
#define CMD_IAP_TRANS   0xA2		//IAP写固件指令
#define CMD_IAP_VERIFY  0xA3		//IAP校验固件指令
#define CMD_IAP_RESET   0xA4		//系统复位指令
#define CMD_IAP_ACK     0xA5       //固件下载应答帧
#define CMD_IAP_READY   0xA6        //固件下载准备


typedef enum {
    IAP_OK				= 0x00 ,
    IAPERROR_SIZE		= 0x01 ,
    IAPERROR_ERASE		= 0x02 , 
    IAPERROR_WRITEFLASH = 0x03 , 
    IAPERROR_UNLOCK		= 0x04 , 
    IAPERROR_INDEX		= 0x05 , 
    IAPERROR_BUSY		= 0x06 , 
    IAPERROR_FORM		= 0x07 , 
    IAPERROR_CRC		= 0x08 , 
    IAPERROR_OTHER		= 0x09 ,
    IAPERROR_HEAD		= 0x10 ,
}IAP_STATUS;

void boot(void);

#endif
