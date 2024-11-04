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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SERVO_TX_Pin GPIO_PIN_2
#define SERVO_TX_GPIO_Port GPIOA
#define SERVO_RX_Pin GPIO_PIN_3
#define SERVO_RX_GPIO_Port GPIOA
#define CSN_Pin GPIO_PIN_4
#define CSN_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_1
#define IRQ_GPIO_Port GPIOB
#define bujin_PWM_Pin GPIO_PIN_9
#define bujin_PWM_GPIO_Port GPIOE
#define HUIDU_5_Pin GPIO_PIN_11
#define HUIDU_5_GPIO_Port GPIOE
#define bujin_CON_Pin GPIO_PIN_13
#define bujin_CON_GPIO_Port GPIOE
#define JY_CON_Pin GPIO_PIN_11
#define JY_CON_GPIO_Port GPIOB
#define JY901s_TX_Pin GPIO_PIN_8
#define JY901s_TX_GPIO_Port GPIOD
#define JY901s_RX_Pin GPIO_PIN_9
#define JY901s_RX_GPIO_Port GPIOD
#define k210_TX_Pin GPIO_PIN_9
#define k210_TX_GPIO_Port GPIOA
#define k210_RX_Pin GPIO_PIN_10
#define k210_RX_GPIO_Port GPIOA
#define ZDT_TX_Pin GPIO_PIN_10
#define ZDT_TX_GPIO_Port GPIOC
#define ZDT_RX_Pin GPIO_PIN_11
#define ZDT_RX_GPIO_Port GPIOC
#define TJC_TX_Pin GPIO_PIN_12
#define TJC_TX_GPIO_Port GPIOC
#define TJC_RX_Pin GPIO_PIN_2
#define TJC_RX_GPIO_Port GPIOD
#define pump_1_Pin GPIO_PIN_3
#define pump_1_GPIO_Port GPIOD
#define pump_2_Pin GPIO_PIN_4
#define pump_2_GPIO_Port GPIOD
#define HOLD_1_Pin GPIO_PIN_5
#define HOLD_1_GPIO_Port GPIOD
#define HOLD_2_Pin GPIO_PIN_6
#define HOLD_2_GPIO_Port GPIOD
#define JQ_C1_Pin GPIO_PIN_7
#define JQ_C1_GPIO_Port GPIOD
#define JQ_C2_Pin GPIO_PIN_9
#define JQ_C2_GPIO_Port GPIOG
#define JQ_C3_Pin GPIO_PIN_10
#define JQ_C3_GPIO_Port GPIOG
#define JQ_C4_Pin GPIO_PIN_11
#define JQ_C4_GPIO_Port GPIOG
#define JY901S_C5_Pin GPIO_PIN_12
#define JY901S_C5_GPIO_Port GPIOG
#define JY901_C6_Pin GPIO_PIN_13
#define JY901_C6_GPIO_Port GPIOG
#define JY901_C7_Pin GPIO_PIN_14
#define JY901_C7_GPIO_Port GPIOG
#define HUIDU_1_Pin GPIO_PIN_3
#define HUIDU_1_GPIO_Port GPIOB
#define HUIDU_2_Pin GPIO_PIN_4
#define HUIDU_2_GPIO_Port GPIOB
#define HUIDU_3_Pin GPIO_PIN_5
#define HUIDU_3_GPIO_Port GPIOB
#define HUIDU_4_Pin GPIO_PIN_6
#define HUIDU_4_GPIO_Port GPIOB
#define LEFT_5_Pin GPIO_PIN_7
#define LEFT_5_GPIO_Port GPIOB
#define LEFT_6_Pin GPIO_PIN_8
#define LEFT_6_GPIO_Port GPIOB
#define LEFT_7_Pin GPIO_PIN_9
#define LEFT_7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
