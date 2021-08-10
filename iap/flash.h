/**
 * @file    flash.h
 * @author  Ferenc Nemeth
 * @date    21 Dec 2018
 * @brief   This module handles the memory related functions.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "main.h"
#include "config.h"

/* Status report for the functions. */
typedef enum {
  FLASH_OK              = 0x00u, /**< The action was successful. */
  FLASH_ERROR_SIZE      = 0x01u, /**< The binary is too big. */
  FLASH_ERROR_WRITE     = 0x02u, /**< Writing failed. */
  FLASH_ERROR_READBACK  = 0x04u, /**< Writing was successful, but the content of the memory is wrong. */
  FLASH_ERROR           = 0xFFu  /**< Generic error. */
} flash_status;

flash_status flash_erase_app(void);
flash_status flash_erase_sector(uint8_t sector_num);
flash_status flash_write(uint32_t address, uint8_t *data, uint32_t length);

void flash_eeprom_load(void);
void flash_eeprom_save(void);
void flash_jump_to_app(void);

#endif /* FLASH_H_ */
