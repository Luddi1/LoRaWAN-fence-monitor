/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_comp.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_lptim.h"
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx.h"
#include "stm32l0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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

//#define APP_DEBUG

#ifdef APP_DEBUG 
    #define MYPRINT(x) do {usartPrint((unsigned char *)x, strlen(x)); usartFlush();} while(0);
#else
    #define MYPRINT(x) do {} while(0);
#endif // APP_DEBUG

// restore lmic from eeprom for debugging after reset, 
// also saves eeprom write cycles and join nonces when developing
#define DEBUG_GET_LMIC_FROM_EEPROM  0

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void incTick(void);
void setTick(uint32_t new_tick);
uint32_t getTick(void);
uint32_t getCurrentMicro(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_IN0_FENCE_Pin LL_GPIO_PIN_0
#define ADC_IN0_FENCE_GPIO_Port GPIOA
#define RFM95_RST_Pin LL_GPIO_PIN_11
#define RFM95_RST_GPIO_Port GPIOA
#define DEBUG_PA12_Pin LL_GPIO_PIN_12
#define DEBUG_PA12_GPIO_Port GPIOA
#define SPI1_NSS_Pin LL_GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
#define DIO0_Pin LL_GPIO_PIN_3
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI2_3_IRQn
#define DIO1_Pin LL_GPIO_PIN_4
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI4_15_IRQn
#define DIO2_Pin LL_GPIO_PIN_5
#define DIO2_GPIO_Port GPIOB
#define DIO2_EXTI_IRQn EXTI4_15_IRQn
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
