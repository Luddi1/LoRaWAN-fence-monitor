/**
  ******************************************************************************
  * File Name          : EEPROM.h
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef EEPROM_H
#define EEPROM_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "arduino_lmic.h"
#include "hal/hal.h"
#include "fence.h"

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define FLASH_PEKEY1               ((uint32_t)0x89ABCDEFU) /*!< Flash program erase key1 */
#define FLASH_PEKEY2               ((uint32_t)0x02030405U) /*!< Flash program erase key: used with FLASH_PEKEY2
                                                               to unlock the write access to the FLASH_PECR register and
                                                               data EEPROM */

// Values for STM32L071KBT, category 6 device, 6kybtes eeprom
#define DATA_E2_ADDR ((uint32_t)0x08080000) /*!< Start of EEPROM Bank 1 */
#define DATA_E2_END  ((uint32_t)0x080817FF) /*!< End of EEPROM Bank 2 */

#define DATA_E2_SIZE            (DATA_E2_END - DATA_E2_ADDR + 1)   /*!< Size of EEPROM in bytes */

/* Number of individual instances in EEPROM for wear leveling = floor( eeprom size / struct size) */
#define DATA_E2_NUM_INSTANCES   (DATA_E2_SIZE / sizeof(struct eeprom_s))

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/*
 * Data that is written to eeprom
 */
struct eeprom_s {
    uint32_t valid;
    struct lmic_t lmic_instance;
    uint32_t tick;
    struct hal_s hal_instance;
    struct fence_s fence_instance;
};

uint8_t eeprom_save(uint32_t tick_save);
uint8_t eeprom_restore_lmic(void);
uint8_t eeprom_restore_fence(void);
uint8_t eeprom_clear(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ EEPROM_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
