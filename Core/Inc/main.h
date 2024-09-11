/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"
#include "../../Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.h"
#include "../../Drivers/Device_Drivers/24C32_EEPROM/24C32.h"
#include "../../Drivers/Device_Drivers/DS1307/DS1307.h"
#include "../../Drivers/Device_Drivers/Matrix_Keypad/Matrix_Keypad.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define kp_button_1 0
#define kp_button_2 1
#define kp_button_3 2
#define kp_button_next 3
#define kp_button_4 4
#define kp_button_5 5
#define kp_button_6 6
#define kp_button_previous 7
#define kp_button_7 8
#define kp_button_8 9
#define kp_button_9 10
#define kp_button_force_feed 11
#define kp_button_save_menu 12
#define kp_button_0 13
#define kp_button_no_back 14
#define kp_button_yes 15



#define dosing_time_hours memory.i2c_buffer[0]
#define dosing_time_minutes memory.i2c_buffer[1]
#define dosing_time_seconds memory.i2c_buffer[2]
#define doses_number memory.i2c_buffer[3]
#define dosing_12_24	memory.i2c_buffer[4]
#define dosing_12 1
#define dosing_24 0
#define dosing_AM_PM	memory.i2c_buffer[5]
#define dosing_PM	0
#define dosing_AM	1
#define dosing_period	memory.i2c_buffer[7]

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
