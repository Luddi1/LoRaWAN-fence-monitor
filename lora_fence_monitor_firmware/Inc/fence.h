/**
  ******************************************************************************
  * File Name          : ADC.h
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
#ifndef FENCE_H
#define FENCE_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include "adc.h"
#include "comp.h"

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define ADC_BUFSIZE 600

struct fence_s {
    uint32_t check_interval;        // in seconds, value for RTC wakeups
    uint32_t heartbeat_counter;     // counter, use in conjunction w/ check_interval to get heartbeat time
    uint32_t fence_timeout;         // timeout value for two fence impulses in ms
    uint32_t voltage_threshold;     // threshold voltage in V, report if below
};

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

int8_t fence_start(void);
uint8_t fence_done(void);
uint8_t fence_need_report(void);
int32_t fence_get(void);
void fence_dma_done_callback(void);
void fence_ComparatorTrigger_Callback(void);
struct fence_s fence_get_persistence(void);
void fence_set_persistence(struct fence_s fence_inst);
void fence_parse_rx(uint8_t *bytes, uint8_t len);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ FENCE_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
