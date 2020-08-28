/**
  ******************************************************************************
  * File Name          : ADC.c
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

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* Timeout values for ADC operations. */
/* (calibration, enable settling time, disable settling time, ...)          */
/* Values defined to be higher than worst cases: low clock frequency,       */
/* maximum prescalers.                                                      */
/* Example of profile very low frequency : ADC clock frequency 0.14MHz      */
/* prescaler 256, sampling time 160.5 ADC clock cycles, resolution 12 bits. */
/*  - ADC calibration time: On STM32L0 ADC, maximum delay is 83/fADC,       */
/*    resulting in a maximum delay of 593us                                 */
/*    (refer to device datasheet, parameter "tCAL")                         */
/*  - ADC enable time: maximum delay is 1 conversion cycle.                 */
/*    (refer to device datasheet, parameter "tSTAB")                        */
/*  - ADC disable time: maximum delay should be a few ADC clock cycles      */
/*  - ADC stop conversion time: maximum delay should be a few ADC clock     */
/*    cycles                                                                */
/*  - ADC conversion time: with this hypothesis of clock settings, maximum  */
/*    delay will be 293ms.                                                  */
/*    (refer to device reference manual, section "Timing")                  */
/* Unit: ms                                                                 */
#define ADC_CALIBRATION_TIMEOUT_MS ((uint32_t)1)
#define ADC_ENABLE_TIMEOUT_MS ((uint32_t)1)
#define ADC_DISABLE_TIMEOUT_MS ((uint32_t)1)
#define ADC_STOP_CONVERSION_TIMEOUT_MS ((uint32_t)1)
#define ADC_CONVERSION_TIMEOUT_MS ((uint32_t)300)

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

/* Definitions of data related to this example */
/* ADC unitary conversion timeout */
/* Considering ADC settings, duration of 1 ADC conversion should always    */
/* be lower than 1ms.                                                      */
#define ADC_UNITARY_CONVERSION_TIMEOUT_MS ((uint32_t)1)

/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

/* Variables for ADC conversion data computation to physical values */
__IO uint16_t uhADCxConvertedData = 0;

// set by adc_calibrate() to the actual VDDA voltage
uint32_t adc_vrefplus = 0;

/* USER CODE END 0 */

/* ADC init function */
void MX_ADC_Init(void)
{
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_InitTypeDef ADC_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    /**ADC GPIO Configuration  
  PA0   ------> ADC_IN0 
  */
    GPIO_InitStruct.Pin = ADC_IN0_FENCE_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(ADC_IN0_FENCE_GPIO_Port, &GPIO_InitStruct);

    /* ADC DMA Init */

    /* ADC Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_0);

    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

    /* ADC interrupt Init */
    NVIC_SetPriority(ADC1_COMP_IRQn, 0);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);

    /** Configure Regular Channel 
  */
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
    /** Common config 
  */
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
    LL_ADC_DisableIT_EOC(ADC1);
    LL_ADC_DisableIT_EOS(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);
    ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);
}

/* USER CODE BEGIN 1 */

void adc_prepare(uint16_t *dest, uint16_t size)
{
    __IO uint32_t wait_loop_index = 0;
    __IO uint32_t backup_setting_adc_dma_transfer = 0;
#if (USE_TIMEOUT == 1)
    uint32_t Timeout = 0; /* Variable used for timeout management */
#endif                    /* USE_TIMEOUT */

    // setup DMA
    /* Set DMA transfer addresses of source and destination */
    LL_DMA_ConfigAddresses(DMA1,
                           LL_DMA_CHANNEL_1,
                           LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                           (uint32_t)dest,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    /* Set DMA transfer size */
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, size);
    /* Enable DMA transfer interruption: transfer complete */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    /* Enable DMA transfer interruption: half transfer */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
    /* Enable DMA transfer interruption: transfer error */
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
    /* Enable the DMA transfer */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    // Enable ADC
    if (LL_ADC_IsEnabled(ADC1) == 0)
    {
        /* Disable ADC DMA transfer request during calibration */
        /* Note: Specificity of this STM32 serie: Calibration factor is           */
        /*       available in data register and also transfered by DMA.           */
        /*       To not insert ADC calibration factor among ADC conversion data   */
        /*       in array variable, DMA transfer must be disabled during          */
        /*       calibration.                                                     */
        backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
        LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

        /* Run ADC self calibration */
        LL_ADC_StartCalibration(ADC1);

/* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

        while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
        {
#if (USE_TIMEOUT == 1)
            /* Check Systick counter flag to decrement the time-out value */
            if (LL_SYSTICK_IsActiveCounterFlag())
            {
                if (Timeout-- == 0)
                {
                    /* Time-out occurred. Set LED to blinking mode */
                    LED_Blinking(LED_BLINK_ERROR);
                }
            }
#endif /* USE_TIMEOUT */
        }

        /* Delay between ADC end of calibration and ADC enable.                   */
        /* Note: Variable divided by 2 to compensate partially                    */
        /*       CPU processing cycles (depends on compilation optimization).     */
        wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
        while (wait_loop_index != 0)
        {
            wait_loop_index--;
        }

        /* Restore ADC DMA transfer request after calibration */
        LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);

        /* Enable ADC */
        LL_ADC_Enable(ADC1);

/* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_ENABLE_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

        while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
        {
#if (USE_TIMEOUT == 1)
            /* Check Systick counter flag to decrement the time-out value */
            if (LL_SYSTICK_IsActiveCounterFlag())
            {
                if (Timeout-- == 0)
                {
                    /* Time-out occurred. Set LED to blinking mode */
                    LED_Blinking(LED_BLINK_ERROR);
                }
            }
#endif /* USE_TIMEOUT */
        }

        /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
        /*       status afterwards.                                               */
        /*       This flag should be cleared at ADC Deactivation, before a new    */
        /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
    }

    return;
}

void adc_start_burst(void)
{

    if ((LL_ADC_IsEnabled(ADC1) == 1) &&
        (LL_ADC_IsDisableOngoing(ADC1) == 0) &&
        (LL_ADC_REG_IsConversionOngoing(ADC1) == 0))
    {
        LL_ADC_REG_StartConversion(ADC1);
    }
    else
    {
        /* Error: ADC conversion start could not be performed */
    }

    return;
}

// to save power when not needed
void adc_disable(void)
{
    /* Peripheral clock disable */
    LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);
}

// returns measured vref, call adc_calibrate() before
uint32_t adc_get_vrefplus(void)
{
    return adc_vrefplus;
}

void adc_calibrate(void)
{
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    __IO uint32_t wait_loop_index = 0;
#if (USE_TIMEOUT == 1)
    uint32_t Timeout = 0; /* Variable used for timeout management */
#endif                    /* USE_TIMEOUT */

    LL_ADC_DeInit(ADC1);

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    /* ADC interrupt Init */
    NVIC_SetPriority(ADC1_COMP_IRQn, 0);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);

    /** Configure Regular Channel 
  */
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_VREFINT);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);
    /** Common config 
  */
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_39CYCLES_5);
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
    LL_ADC_DisableIT_EOC(ADC1);
    LL_ADC_DisableIT_EOS(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);
    ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);

    /*## Operation on ADC hierarchical scope: ADC instance #####################*/
    if (LL_ADC_IsEnabled(ADC1) == 0)
    {
        /* Run ADC self calibration */
        LL_ADC_StartCalibration(ADC1);

/* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

        while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
        {
#if (USE_TIMEOUT == 1)
            /* Check Systick counter flag to decrement the time-out value */
            if (LL_SYSTICK_IsActiveCounterFlag())
            {
                if (Timeout-- == 0)
                {
                    /* Time-out occurred. Set LED to blinking mode */
                }
            }
#endif /* USE_TIMEOUT */
        }

        /* Delay between ADC end of calibration and ADC enable.                   */
        /* Note: Variable divided by 2 to compensate partially                    */
        /*       CPU processing cycles (depends on compilation optimization).     */
        wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
        while (wait_loop_index != 0)
        {
            wait_loop_index--;
        }

        /* Enable ADC */
        LL_ADC_Enable(ADC1);

/* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_ENABLE_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

        while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
        {
#if (USE_TIMEOUT == 1)
            /* Check Systick counter flag to decrement the time-out value */
            if (LL_SYSTICK_IsActiveCounterFlag())
            {
                if (Timeout-- == 0)
                {
                    /* Time-out occurred. Set LED to blinking mode */
                    LED_Blinking(LED_BLINK_ERROR);
                }
            }
#endif /* USE_TIMEOUT */
        }

        /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
        /*       status afterwards.                                               */
        /*       This flag should be cleared at ADC Deactivation, before a new    */
        /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
    }

    if ((LL_ADC_IsEnabled(ADC1) == 1) &&
        (LL_ADC_IsDisableOngoing(ADC1) == 0) &&
        (LL_ADC_REG_IsConversionOngoing(ADC1) == 0))
    {
        LL_ADC_REG_StartConversion(ADC1);
    }
    else
    {
        /* Error: ADC conversion start could not be performed */
    }

#if (USE_TIMEOUT == 1)
    Timeout = ADC_UNITARY_CONVERSION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

    while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0)
    {
#if (USE_TIMEOUT == 1)
        /* Check Systick counter flag to decrement the time-out value */
        if (LL_SYSTICK_IsActiveCounterFlag())
        {
            if (Timeout-- == 0)
            {
                /* Time-out occurred. Set LED to blinking mode */
                LED_Blinking(LED_BLINK_SLOW);
            }
        }
#endif /* USE_TIMEOUT */
    }

    /* Clear flag ADC group regular end of unitary conversion */
    LL_ADC_ClearFlag_EOC(ADC1);

    /* Retrieve ADC conversion data */
    /* (data scale corresponds to ADC resolution: 12 bits) */
    uhADCxConvertedData = LL_ADC_REG_ReadConversionData12(ADC1);

    adc_vrefplus = __LL_ADC_CALC_VREFANALOG_VOLTAGE(uhADCxConvertedData, LL_ADC_RESOLUTION_12B);

    // set ADC to off before leaving
    adc_disable();
    LL_ADC_DeInit(ADC1);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
