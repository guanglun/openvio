/**
 * @file    flash.c
 * @author  Ferenc Nemeth
 * @date    21 Dec 2018
 * @brief   This module handles the memory related functions.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#include "flash.h"
#include "Legacy/stm32_hal_legacy.h"

struct EEPROM_CONFIG_STRUCT eeprom;

/* Function pointer for jumping to user application. */
typedef void (*fnc_ptr)(void);

flash_status flash_erase_app(void)
{

  uint8_t sector_num = FLASH_APP_START_SECTOR;
  for (; sector_num <= FLASH_APP_END_SECTOR; sector_num++)
  {
    if (flash_erase_sector(sector_num) != FLASH_OK)
    {
      return FLASH_ERROR;
    }
  }
  return FLASH_OK;
}

flash_status flash_erase_sector(uint8_t sector_num)
{
  HAL_FLASH_Unlock();

  flash_status status = FLASH_ERROR;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t error = 0u;

  erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init.NbSectors = 1;
  erase_init.Sector = sector_num;
  erase_init.Banks = FLASH_BANK_1;
  erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  /* Do the actual erasing. */
  if (HAL_OK == HAL_FLASHEx_Erase(&erase_init, &error))
  {
    status = FLASH_OK;
  }

  HAL_FLASH_Lock();

  return status;
}

/**
 * @brief   This function flashes the memory.
 * @param   address: First address to be written to.
 * @param   *data:   Array of the data that we want to write.
 * @param   *length: Size of the array.
 * @return  status: Report about the success of the writing.
 */
flash_status flash_write(uint32_t address, uint8_t *data, uint32_t length)
{
  flash_status status = FLASH_OK;
  uint64_t FlashWord[4];

  __set_PRIMASK(1); /* 关中断 */

  HAL_FLASH_Unlock();

  /* Loop through the array. */
  for (uint32_t i = 0u; (i < length / 32) && (FLASH_OK == status); i++)
  {
    /* If we reached the end of the memory, then report an error and don't do anything else.*/
    if (FLASH_APP_END_ADDRESS <= address)
    {
      status |= FLASH_ERROR_SIZE;
    }
    else
    {
      memcpy((char *)FlashWord, data, 32);
      data += 32;
      /* The actual flashing. If there is an error, then report it. */
      if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, FlashWord))
      {
        status |= FLASH_ERROR_WRITE;
      }
      /* Shift the address by a word. */
      address += 32u;
    }
  }

  if (length % 32)
  {
    FlashWord[0] = 0;
    FlashWord[1] = 0;
    FlashWord[2] = 0;
    FlashWord[3] = 0;

    memcpy((char *)FlashWord, data, 32);
    /* The actual flashing. If there is an error, then report it. */
    if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, FlashWord))
    {
      status |= FLASH_ERROR_WRITE;
    }
  }

  HAL_FLASH_Lock();
  __set_PRIMASK(0); /* 开中断 */
  return status;
}

/**
 * @brief   Actually jumps to the user application.
 * @param   void
 * @return  void
 */
void flash_jump_to_app(void)
{
  /* Function pointer to the address of the user application. */
  fnc_ptr jump_to_app;
  jump_to_app = (fnc_ptr)(*(volatile uint32_t *)(FLASH_APP_START_ADDRESS + 4u));
  HAL_DeInit();
  /* Change the main stack pointer. */
  __set_MSP(*(volatile uint32_t *)FLASH_APP_START_ADDRESS);
  jump_to_app();
}


void flash_eeprom_save(void)
{
  uint64_t FlashWord[4]={0,0,0,0};

  flash_status status = FLASH_ERROR;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t error = 0u;

  erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init.NbSectors = 1;
  erase_init.Sector = FLASH_SECTOR_0;
  erase_init.Banks = FLASH_BANK_2;
  erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  memcpy((char *)FlashWord, &eeprom, sizeof(struct EEPROM_CONFIG_STRUCT));

  HAL_FLASH_Unlock();

  /* Do the actual erasing. */
  if (HAL_OK == HAL_FLASHEx_Erase(&erase_init, &error))
  {
    status = FLASH_OK;
  }


  if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, FLASH_EEPROM_START_ADDRESS, FlashWord))
  {
    status |= FLASH_ERROR_WRITE;
  }

  HAL_FLASH_Lock();

  //printf("flash save success\r\n");
}

void flash_eeprom_load(void)
{
  memcpy(&eeprom, FLASH_EEPROM_START_ADDRESS, sizeof(struct EEPROM_CONFIG_STRUCT));
}