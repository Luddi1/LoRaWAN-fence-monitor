/**
  ******************************************************************************
  * File Name          : RTC.c
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* Value defined for WUT */
#define RTC_WUT_TIME               ((uint32_t)5)     /* 5 s */

/* Oscillator time-out values */
#define LSI_TIMEOUT_VALUE          ((uint32_t)100)   /* 100 ms */
#define LSE_TIMEOUT_VALUE          ((uint32_t)5000)  /* 5 s */
#define RTC_TIMEOUT_VALUE          ((uint32_t)1000)  /* 1 s */

/* Defines related to Clock configuration */
/* Uncomment to enable the adequate Clock Source */
#define RTC_CLOCK_SOURCE_LSI
/*#define RTC_CLOCK_SOURCE_LSE*/

#ifdef RTC_CLOCK_SOURCE_LSI
/* ck_apre=LSIFreq/(ASYNC prediv + 1) = 256Hz with LSIFreq=37 kHz RC */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
/* ck_spre=ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV           ((uint32_t)0x122)
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
/* ck_apre=LSEFreq/(ASYNC prediv + 1) = 256Hz with LSEFreq=32768Hz */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
/* ck_spre=ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00FF)
#endif

#if (USE_TIMEOUT == 1)
uint32_t Timeout = 0; /* Variable used for Timeout management */
#endif /* USE_TIMEOUT */

// backup register defines
#define RTC_BKPR_HEARTBEAT_MASK (0x7FFFFFFF)
#define RTC_BKPR_ALARMFLAG_MASK (0x80000000)


// Function prototypes
static uint32_t rtc_reg_get(void);
static void rtc_reg_set(uint32_t hb);

/* USER CODE END 0 */

/* RTC init function */
void MX_RTC_Init(void)
{
  LL_RTC_InitTypeDef RTC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

  /* RTC interrupt Init */
  NVIC_SetPriority(RTC_IRQn, 0);
  NVIC_EnableIRQ(RTC_IRQn);

  /** Initialize RTC and set the Time and Date 
  */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  /** Initialize RTC and set the Time and Date 
  */
  /** Enable the WakeUp 
  */
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_DIV_16);

}

/* USER CODE BEGIN 1 */

/*
 * Increases hearbeat register in RTC. 
 * Returns non-zero if heartbeat required
 * and resets counter and alarm report flag. 
 * @param   hb_compare if value in register >= param then 
 *          schedule heartbeat
 */
uint8_t rtc_heartbeat_get(uint32_t hb_compare)
{
    uint8_t report;
    uint32_t regval = rtc_reg_get() & RTC_BKPR_HEARTBEAT_MASK;

    if (regval >= hb_compare)
    {
        // reset heartbeat counter and alarm report flag
        rtc_reg_set(0);
        report = 1;
    }
    else
    {
        // increase heartbeat counter, preserve alarm flag
        rtc_reg_set( ((regval + 1) & RTC_BKPR_HEARTBEAT_MASK) | rtc_alarm_reported_flag_get() );
        report = 0;
    }

    return report;
}

/*
 * Returns non-zero if alarm already reported
 * else 0.
 */
uint32_t rtc_alarm_reported_flag_get(void)
{
    uint32_t regval = rtc_reg_get() & RTC_BKPR_ALARMFLAG_MASK;

    return regval;
}

/*
 * Sets flag to 1
 */
void rtc_alarm_reported_flag_set(void)
{
    uint32_t regval = rtc_reg_get();
    rtc_reg_set(regval | RTC_BKPR_ALARMFLAG_MASK);
}

/**
  * Brief   This function configures RTC.
  * Param   None
  * Retval  None
  */
void Configure_RTC(uint32_t rtc_wut)
{
  /*##-1- Enables the PWR Clock and Enables access to the backup domain #######*/
  /* To change the source clock of the RTC feature (LSE, LSI), you have to:
     - Enable the power clock
     - Enable write access to configure the RTC clock source (to be done once after reset).
     - Reset the Back up Domain
     - Configure the needed RTC clock source */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();
  

  /* Enable LSI */
  LL_RCC_LSI_Enable();
#if (USE_TIMEOUT == 1)
  Timeout = LSI_TIMEOUT_VALUE;
#endif /* USE_TIMEOUT */
  while (LL_RCC_LSI_IsReady() != 1)
  {
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag()) 
    {
      Timeout --;
    }
    if (Timeout == 0)
    {
      /* LSI activation error */
      //LED_Blinking(LED_BLINK_ERROR);
    }  
#endif /* USE_TIMEOUT */
  }

  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  
  /*##-3- Enable RTC peripheral Clocks #######################################*/
  /* Enable RTC Clock */ 
  LL_RCC_EnableRTC();


  /*##-4- Configure RTC ######################################################*/
  /* Disable RTC registers write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Set prescaler according to source clock */
  LL_RTC_SetAsynchPrescaler(RTC, RTC_ASYNCH_PREDIV);
  LL_RTC_SetSynchPrescaler(RTC, RTC_SYNCH_PREDIV);

  /* Disable wake up timer to modify it */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* Wait until it is allow to modify wake up reload value */
#if (USE_TIMEOUT == 1)
  Timeout = RTC_TIMEOUT_VALUE;
#endif /* USE_TIMEOUT */

  while (LL_RTC_IsActiveFlag_WUTW(RTC) != 1)
  {
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      Timeout --;
    }
    if (Timeout == 0)
    {
      /* LSI activation error */
      //LED_Blinking(LED_BLINK_ERROR);
    }  
#endif /* USE_TIMEOUT */
  }

  /* Setting the Wakeup time to RTC_WUT_TIME s
       If LL_RTC_WAKEUPCLOCK_CKSPRE is selected, the frequency is 1Hz, 
       this allows to get a wakeup time equal to RTC_WUT_TIME s 
       if the counter is RTC_WUT_TIME */
  LL_RTC_WAKEUP_SetAutoReload(RTC, rtc_wut);
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
  
  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);
  
}

static uint32_t rtc_reg_get(void)
{
    return LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0);
}

static void rtc_reg_set(uint32_t hb)
{
    // DBP bit must be set in order to enable RTC registers write access.
    LL_PWR_EnableBkUpAccess();

    // Disable write protection and set register
    LL_RTC_DisableWriteProtection(RTC);
    LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, hb);
    LL_RTC_EnableWriteProtection(RTC);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
