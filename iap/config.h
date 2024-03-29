#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define VERSION1 0
#define VERSION2 0
#define VERSION3 5

#define JUMP_APP_DELAY  1000

#define FLASH_BOOTLOADER_START_SECTOR           FLASH_SECTOR_0
#define FLASH_BOOTLOADER_END_SECTOR             FLASH_SECTOR_0 
#define FLASH_APP_START_SECTOR                  FLASH_SECTOR_1
#define FLASH_APP_END_SECTOR                    FLASH_SECTOR_7    

//bootloader 128k 
#define FLASH_BOOTLOADER_START_ADDRESS  ((uint32_t)FLASH_BASE)
#define FLASH_BOOTLOADER_END_ADDRESS    ((uint32_t)0x08020000u-1)  
/* Start and end addresses of the user application. */
#define FLASH_APP_START_ADDRESS         ((uint32_t)0x08020000u)
#define FLASH_APP_END_ADDRESS           ((uint32_t)FLASH_BANK2_BASE-1)
#define FLASH_EEPROM_START_ADDRESS      ((uint32_t)FLASH_BANK2_BASE)


struct EEPROM_CONFIG_STRUCT{
  uint32_t reboot_to_bootloader;
  int32_t exposure;
  uint8_t is_sync_mode;
  uint8_t camera_fps;
  uint8_t infrared_pwm;
}__attribute__ ((aligned (4)));;


#ifdef __cplusplus
}
#endif

#endif
