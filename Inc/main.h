/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {FALSE = 0,TRUE = 1} bool;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void get_time(uint32_t *t1_cnt,uint16_t *t2_cnt);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOC
#define USB_ENABLE_Pin GPIO_PIN_7
#define USB_ENABLE_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_4
#define IMU_INT_GPIO_Port GPIOC
#define IMU_INT_EXTI_IRQn EXTI4_IRQn
#define IMU_FSYNC_Pin GPIO_PIN_5
#define IMU_FSYNC_GPIO_Port GPIOC
#define INPUT_E7_Pin GPIO_PIN_7
#define INPUT_E7_GPIO_Port GPIOE
#define INPUT_E8_Pin GPIO_PIN_8
#define INPUT_E8_GPIO_Port GPIOE
#define LED_E9_Pin GPIO_PIN_9
#define LED_E9_GPIO_Port GPIOE
#define LED_E10_Pin GPIO_PIN_10
#define LED_E10_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_12
#define KEY_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_14
#define LED_B_GPIO_Port GPIOE
#define USB_SWITCH_Pin GPIO_PIN_15
#define USB_SWITCH_GPIO_Port GPIOE
#define SYNC_CLOCK_Pin GPIO_PIN_15
#define SYNC_CLOCK_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_10
#define IMU_SDA_GPIO_Port GPIOD
#define IMU_SCL_Pin GPIO_PIN_11
#define IMU_SCL_GPIO_Port GPIOD
#define DCMI_RST_Pin GPIO_PIN_10
#define DCMI_RST_GPIO_Port GPIOA
#define TEST1_Pin GPIO_PIN_13
#define TEST1_GPIO_Port GPIOA
#define SD_CD_Pin GPIO_PIN_0
#define SD_CD_GPIO_Port GPIOD
#define IMU_SPI_CS_Pin GPIO_PIN_1
#define IMU_SPI_CS_GPIO_Port GPIOD
#define CAMSYNC_Pin GPIO_PIN_3
#define CAMSYNC_GPIO_Port GPIOD
#define CAMSYNC_EXTI_IRQn EXTI3_IRQn
#define TFT_SPI_CS_Pin GPIO_PIN_4
#define TFT_SPI_CS_GPIO_Port GPIOD
#define DCMI_PWDN_Pin GPIO_PIN_7
#define DCMI_PWDN_GPIO_Port GPIOD
#define DCMI_FSYNC_Pin GPIO_PIN_4
#define DCMI_FSYNC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
