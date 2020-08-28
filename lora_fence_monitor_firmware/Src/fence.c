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

/* Includes ------------------------------------------------------------------*/
#include "fence.h"
#include "string.h"

static struct fence_s fence_vars;
static uint16_t adc_buf[ADC_BUFSIZE] = {0};
static volatile uint8_t measurement_ongoing = 0;
static volatile uint8_t report_needed = 0;
static volatile uint8_t second_trigger = 0;

// from https://rosettacode.org/wiki/Sorting_algorithms/Shell_sort#C
static void shell_sort(int *a, int n)
{
    int h, i, j, t;
    for (h = n; h /= 2;)
    {
        for (i = h; i < n; i++)
        {
            t = a[i];
            for (j = i; j >= h && t < a[j - h]; j -= h)
            {
                a[j] = a[j - h];
            }
            a[j] = t;
        }
    }
}

int8_t fence_start(void)
{
    measurement_ongoing = 1;
    report_needed = 0;

    /* Enable comparator and start ADC */
    adc_prepare(adc_buf, ADC_BUFSIZE);
    LL_mDelay(5);
    LL_COMP_Enable(COMP1);

    return 0;
}

uint8_t fence_done(void)
{
    return !measurement_ongoing;
}

/*
 * indicates if fence measurement has to be reported
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
    int adc_median_buf[MEDIAN_SIZE];
    uint32_t adc_median;
    uint32_t vref = 0;

    if (measurement_ongoing)
    {
        // not done yet
        return -1;
    }

    for (int i = 0; i < (ADC_BUFSIZE - MEDIAN_SIZE - 1); i += MEDIAN_SIZE / 2)
    {
        // get median over MEDIAN_SIZE samples
        for (int j = 0; j < MEDIAN_SIZE; j++)
        {
            adc_median_buf[j] = (int)adc_buf[i + j];
        }
        shell_sort(adc_median_buf, MEDIAN_SIZE);
        adc_median = (uint32_t)adc_median_buf[(MEDIAN_SIZE - 1U) / 2U];

        // get maximum value
        adcmax = (adc_median > adcmax) ? adc_median : adcmax;
    }

    vref = adc_get_vrefplus();

    report = (vref * adcmax) / (4095); // to voltage at pin in mV
    report = (report * 4256) / 1000;   // to voltage before voltage divider, 4k7 + 20M

    // determine if report necessary
    if (report < fence_vars.voltage_threshold)
    {
        report_needed = 1;
    }

    return report;
}

void fence_dma_done_callback(void)
{
    measurement_ongoing = 0;
}

void fence_ComparatorTrigger_Callback(void)
{
    // Use two fence impulses to synchronize with first impulse,
    // avoids the scenario in which the comparator is enabled during
    // an impulse, immediatley triggers and the maximum voltage measured
    // is too low
    // -> timeout has to account for two impulses
    if (!second_trigger)
    {
        // stop subsequent triggers
        LL_COMP_Disable(COMP1);

        second_trigger = 1;

        // wait for fence impulse to seize
        LL_mDelay(100);

        // stop subsequent triggers
        LL_COMP_Enable(COMP1);
    }
    else
    {
        // stop subsequent triggers
        LL_COMP_Disable(COMP1);

        // sample data
        adc_start_burst();
    }

    return;
}

struct fence_s fence_get_persistence(void)
{
    return fence_vars;
}

void fence_set_persistence(struct fence_s fence_inst)
{
    fence_vars = fence_inst;
}

void fence_parse_rx(uint8_t *bytes, uint8_t len)
{
    if (len != 4) // accepts exactly 4 bytes
    {
        return;
    }

    // first byte check_interval, from minutes to seconds
    fence_vars.check_interval = bytes[0] * 60;

    // second byte heartbeat_counter
    fence_vars.heartbeat_counter = bytes[1];

    // third byte fence_timeout, from 0.1s to ms
    fence_vars.fence_timeout = bytes[2] * 100;

    // fourth byte voltage_threshold, from 100V to V
    fence_vars.voltage_threshold = bytes[3] * 100;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
