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
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRF_SPI5_SCK_Pin GPIO_PIN_7
#define NRF_SPI5_SCK_GPIO_Port GPIOF
#define NRF_SPI5_MISO_Pin GPIO_PIN_8
#define NRF_SPI5_MISO_GPIO_Port GPIOF
#define ESC_TIM14_CH1_Pin GPIO_PIN_9
#define ESC_TIM14_CH1_GPIO_Port GPIOF
#define ENCODER_A_Pin GPIO_PIN_0
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_A_EXTI_IRQn EXTI0_IRQn
#define SERVO_TIM13_CH1_Pin GPIO_PIN_6
#define SERVO_TIM13_CH1_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_0
#define ENCODER_B_GPIO_Port GPIOB
#define NRF_SPI5_MOSI_Pin GPIO_PIN_11
#define NRF_SPI5_MOSI_GPIO_Port GPIOF
#define IMU_I2C4_SCL_Pin GPIO_PIN_14
#define IMU_I2C4_SCL_GPIO_Port GPIOF
#define IMU_I2C4_SDA_Pin GPIO_PIN_15
#define IMU_I2C4_SDA_GPIO_Port GPIOF
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define NRF_CE_Pin GPIO_PIN_6
#define NRF_CE_GPIO_Port GPIOC
#define NRF_CSN_Pin GPIO_PIN_7
#define NRF_CSN_GPIO_Port GPIOC
#define LD2_YELLOW_Pin GPIO_PIN_1
#define LD2_YELLOW_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
