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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RELAY_CTR_2_Pin GPIO_PIN_2
#define RELAY_CTR_2_GPIO_Port GPIOE
#define RELAY_CTR_3_Pin GPIO_PIN_3
#define RELAY_CTR_3_GPIO_Port GPIOE
#define RELAY_CTR_4_Pin GPIO_PIN_4
#define RELAY_CTR_4_GPIO_Port GPIOE
#define RELAY_CTR_5_Pin GPIO_PIN_5
#define RELAY_CTR_5_GPIO_Port GPIOE
#define RELAY_CTR_6_Pin GPIO_PIN_6
#define RELAY_CTR_6_GPIO_Port GPIOE
#define INPUT_DET_0_Pin GPIO_PIN_0
#define INPUT_DET_0_GPIO_Port GPIOF
#define INPUT_DET_1_Pin GPIO_PIN_1
#define INPUT_DET_1_GPIO_Port GPIOF
#define INPUT_DET_2_Pin GPIO_PIN_2
#define INPUT_DET_2_GPIO_Port GPIOF
#define INPUT_DET_3_Pin GPIO_PIN_3
#define INPUT_DET_3_GPIO_Port GPIOF
#define INPUT_DET_4_Pin GPIO_PIN_4
#define INPUT_DET_4_GPIO_Port GPIOF
#define INPUT_DET_5_Pin GPIO_PIN_5
#define INPUT_DET_5_GPIO_Port GPIOF
#define INPUT_DET_6_Pin GPIO_PIN_6
#define INPUT_DET_6_GPIO_Port GPIOF
#define INPUT_DET_7_Pin GPIO_PIN_7
#define INPUT_DET_7_GPIO_Port GPIOF
#define AUX_IN_1_Pin GPIO_PIN_8
#define AUX_IN_1_GPIO_Port GPIOF
#define AUX_IN_2_Pin GPIO_PIN_9
#define AUX_IN_2_GPIO_Port GPIOF
#define RELAY_CTR_7_Pin GPIO_PIN_7
#define RELAY_CTR_7_GPIO_Port GPIOE
#define UART3_TX_LAN_Pin GPIO_PIN_10
#define UART3_TX_LAN_GPIO_Port GPIOB
#define UART3_RX_LAN_Pin GPIO_PIN_11
#define UART3_RX_LAN_GPIO_Port GPIOB
#define EXT_IN_1_LED_Pin GPIO_PIN_8
#define EXT_IN_1_LED_GPIO_Port GPIOD
#define EXT_IN_2_LED_Pin GPIO_PIN_9
#define EXT_IN_2_LED_GPIO_Port GPIOD
#define MCU_STATUS_Pin GPIO_PIN_10
#define MCU_STATUS_GPIO_Port GPIOD
#define LAN_RESET_Pin GPIO_PIN_15
#define LAN_RESET_GPIO_Port GPIOD
#define UART4_TX_PWR_MON_Pin GPIO_PIN_10
#define UART4_TX_PWR_MON_GPIO_Port GPIOC
#define UART4_RX_PWR_MON_Pin GPIO_PIN_11
#define UART4_RX_PWR_MON_GPIO_Port GPIOC
#define INPUT_DET_0_LED_Pin GPIO_PIN_0
#define INPUT_DET_0_LED_GPIO_Port GPIOD
#define INPUT_DET_1_LED_Pin GPIO_PIN_1
#define INPUT_DET_1_LED_GPIO_Port GPIOD
#define INPUT_DET_2_LED_Pin GPIO_PIN_2
#define INPUT_DET_2_LED_GPIO_Port GPIOD
#define INPUT_DET_3_LED_Pin GPIO_PIN_3
#define INPUT_DET_3_LED_GPIO_Port GPIOD
#define INPUT_DET_4_LED_Pin GPIO_PIN_4
#define INPUT_DET_4_LED_GPIO_Port GPIOD
#define INPUT_DET_5_LED_Pin GPIO_PIN_5
#define INPUT_DET_5_LED_GPIO_Port GPIOD
#define INPUT_DET_6_LED_Pin GPIO_PIN_6
#define INPUT_DET_6_LED_GPIO_Port GPIOD
#define INPUT_DET_7_LED_Pin GPIO_PIN_7
#define INPUT_DET_7_LED_GPIO_Port GPIOD
#define TEMP_HUMI_SCK_Pin GPIO_PIN_6
#define TEMP_HUMI_SCK_GPIO_Port GPIOB
#define TEMP_HUMI_SDA_Pin GPIO_PIN_7
#define TEMP_HUMI_SDA_GPIO_Port GPIOB
#define RELAY_CTR_0_Pin GPIO_PIN_0
#define RELAY_CTR_0_GPIO_Port GPIOE
#define RELAY_CTR_1_Pin GPIO_PIN_1
#define RELAY_CTR_1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define EEPROM_IDX_EEPROM_INIT_FLAG		(0)
#define EEPROM_IDX_UPDATE_RSLT			(1)
#define EEPROM_IDX_RESET_REQ				(2)
#define EEPROM_IDX_JUMP_TO_APP_TIMEOUT	(3)
#define EEPROM_IDX_JUMP_REQ				(4)
#define EEPROM_IDX_CHANNEL_RELAY_STATUS	(5)
#define EEPROM_IDX_TEMP_OFFSET_SIGN		(6)
#define EEPROM_IDX_TEMP_OFFSET_VALUE		(7)
#define EEPROM_IDX_HUMID_OFFSET_SIGN		(8)
#define EEPROM_IDX_HUMID_OFFSET_VALUE	(9)

#define PC_ADDR		(0)
#define BOARD_ADDR	(1)

#define STX_PC_TO_BOARD	(0x7E)
#define END_PC_TO_BOARD	(0x81)
#define STX_BOARD_TO_PC	(0x25)
#define END_BOARD_TO_PC	(0x4A)

#define PC_TO_BOARD_REQ_VER							(0xA0)
#define PC_TO_BOARD_REQ_RESET							(0xA1)
#define PC_TO_BOARD_REQ_ERASE_PAGE					(0xA2)
#define PC_TO_BOARD_REQ_WRITE_PAGE					(0xA3)
#define PC_TO_BOARD_REQ_START_UPDATE					(0xA4)
#define PC_TO_BOARD_REQ_JUMP_TO_APP					(0xA5)
#define PC_TO_BOARD_REQ_GET_CHANNEL_POWER			(0xA6)
#define PC_TO_BOARD_REQ_GET_CHANNEL_RELAY			(0xA7)
#define PC_TO_BOARD_REQ_SET_EACH_CHANNEL			(0xA8)
#define PC_TO_BOARD_REQ_SET_ALL_CHANNEL				(0xA9)
#define BOARD_TO_PC_ANS_VER							(0xB0)
#define BOARD_TO_PC_ANS_RESET							(0xB1)
#define BOARD_TO_PC_ANS_ERASE_PAGE					(0xB2)
#define BOARD_TO_PC_ANS_WRITE_PAGE					(0xB3)
#define BOARD_TO_PC_ANS_START_UPDATE					(0xB4)
#define BOARD_TO_PC_ANS_JUMP_TO_APP					(0xB5)
#define BOARD_TO_PC_ANS_GET_CHANNEL_POWER			(0xB6)
#define BOARD_TO_PC_ANS_GET_CHANNEL_RELAY			(0xB7)
#define BOARD_TO_PC_ANS_SET_EACH_CHANNEL_RELAY		(0xB8)
#define BOARD_TO_PC_ANS_SET_ALL_CHANNEL_RELAY		(0xB9)

#define RELAY_OFF	(0)
#define RELAY_ON	(1)

#define UART3_TX_BUF_SIZE	(100)
#define UART3_RX_BUF_SIZE	(100)
#define UART3_DATA_BUF_SIZE	(2064)
#define UART4_RX_BUF_SIZE	(100)

#define REFRESH_IWDG_TIMEOUT		(2000)
#define UART3_PARSING_TIMEOUT		(100)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
