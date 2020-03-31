/**
  ******************************************************************************
  * File Name          : fence.c
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

/**
 * test an oberen draht: länge 270, max bei 1000 -> gallagher 6.4kV
 */

// TODO timeout

/* Includes ------------------------------------------------------------------*/
#include "fence.h"

static uint16_t adc_buf[ADC_BUFSIZE] = {0};
static volatile uint8_t measurement_ongoing = 0;
static volatile uint8_t report_needed = 0;

int8_t fence_start(void)
{
    measurement_ongoing = 1;
    report_needed = 0;

    /* Enable comparator and start ADC */
    LL_COMP_Enable(COMP1);
    adc_prepare(adc_buf, ADC_BUFSIZE);

    return 0;
}

uint8_t fence_done(void)
{
    return !measurement_ongoing;
}

/*
 * indicates if fence measurement has to be reported
 * TODO
 */
uint8_t fence_need_report(void)
{
    return report_needed;
}

/*
 * returns fence voltage in volts
 * -1 on error
 */
int32_t fence_get(void)
{
    uint16_t adcmax = 0;
    int32_t report = 0;

    if (measurement_ongoing)
    {
        // not done yet
        return -1;
    }

    // get maximum value in buffer
    for (int i = 0; i < ADC_BUFSIZE; i++)
    {
        adcmax = (adc_buf[i] > adcmax) ? adc_buf[i] : adcmax;
    }

    // convert to voltage
    report = (adcmax * 64) / 10;

    // determine if report necessary
    if (report < VOLTAGE_THRESH)
    {
        report_needed = 1;
    }

    return report;
}

void fence_dma_done_callback(void)
{
    measurement_ongoing = 0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
