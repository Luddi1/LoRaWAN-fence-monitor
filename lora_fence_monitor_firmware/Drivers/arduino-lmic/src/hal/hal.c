/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * Copyright (c) 2018-2019 MCCI Corporation
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/


// include all the lmic header files, including ../lmic/hal.h
#include "../arduino_lmic.h"
// include the C++ hal.h
#include "hal.h"
// we may need some things from stdio.
#include <stdio.h>

// Includes for STM32 peripherals
#include "gpio.h"
#include "spi.h"
#include "main.h"
#include "lptim.h"

// -----------------------------------------------------------------------------
// I/O

static const HalPinmap_t *plmic_pins;
static hal_failure_handler_t* custom_hal_failure_handler = NULL;

// variales that have to be preserved after sleep
static struct hal_s hal_vars;

static void hal_interrupt_init(); // Fwd declaration
static void delayMicroseconds(uint32_t us);

static void hal_io_init () {
    // done in separate methods in main.c

    hal_interrupt_init();
}

struct hal_s hal_get_persistence(void)
{
    return hal_vars;
}

void hal_set_persistence(struct hal_s hal_inst)
{
    hal_vars = hal_inst;
}

// val == 1  => tx
void hal_pin_rxtx (u1_t val) {
    // if (plmic_pins->rxtx != LMIC_UNUSED_PIN)
    //     digitalWrite(plmic_pins->rxtx, val != plmic_pins->rxtx_rx_active);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    // if (plmic_pins->rst == LMIC_UNUSED_PIN)
    //     return;

    if (val == 1) 
    {   // drive pin
        LL_GPIO_SetPinMode(RFM95_RST_GPIO_Port, RFM95_RST_Pin, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetOutputPin(RFM95_RST_GPIO_Port, RFM95_RST_Pin);
    } else if (val == 0)
    {   // drive pin
        LL_GPIO_SetPinMode(RFM95_RST_GPIO_Port, RFM95_RST_Pin, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(RFM95_RST_GPIO_Port, RFM95_RST_Pin);
    } else 
    {   // floating
        LL_GPIO_SetPinMode(RFM95_RST_GPIO_Port, RFM95_RST_Pin, LL_GPIO_MODE_ANALOG);
    }
}

s1_t hal_getRssiCal (void) {
    return plmic_pins->rssi_cal;
}

#if !defined(LMIC_USE_INTERRUPTS)
static void hal_interrupt_init() {
    // pinMode(plmic_pins->dio[0], INPUT);
    // if (plmic_pins->dio[1] != LMIC_UNUSED_PIN)
    //     pinMode(plmic_pins->dio[1], INPUT);
    // if (plmic_pins->dio[2] != LMIC_UNUSED_PIN)
    //     pinMode(plmic_pins->dio[2], INPUT);
}

static bool dio_states[NUM_DIO] = {0};
static void hal_io_check() {
    // uint8_t i;
    // for (i = 0; i < NUM_DIO; ++i) {
    //     if (plmic_pins->dio[i] == LMIC_UNUSED_PIN)
    //         continue;

    //     if (dio_states[i] != digitalRead(plmic_pins->dio[i])) {
    //         dio_states[i] = !dio_states[i];
    //         if (dio_states[i])
    //             radio_irq_handler(i);
    //     }
    // }
}

#else
// Interrupt handlers
static ostime_t interrupt_time[NUM_DIO] = {0};

void hal_isrPin0(void) {
    ostime_t now = os_getTime();
    interrupt_time[0] = now ? now : 1;
}
void hal_isrPin1(void) {
    ostime_t now = os_getTime();
    interrupt_time[1] = now ? now : 1;
}
void hal_isrPin2(void) {
    ostime_t now = os_getTime();
    interrupt_time[2] = now ? now : 1;
}

static void hal_interrupt_init() {
    // call interrupt handlers from stm32l0xx_it.c
}

static void hal_io_check() {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        ostime_t iTime;
        // if (plmic_pins->dio[i] == LMIC_UNUSED_PIN)
        //     continue;

        iTime = interrupt_time[i];
        if (iTime) {
            interrupt_time[i] = 0;
            radio_irq_handler_v2(i, iTime);
        }
    }
}
#endif // LMIC_USE_INTERRUPTS

// -----------------------------------------------------------------------------
// SPI

static void hal_spi_init () {
    // most of config done in spi.c
}

static void hal_spi_trx(u1_t cmd, u1_t* buf, size_t len, bit_t is_read) {
    uint32_t i;
    u1_t data;

    LL_SPI_Enable(SPI1);
    LL_GPIO_ResetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);

    /* cmd */
    LL_SPI_TransmitData8(SPI1, cmd);

    while (LL_SPI_IsActiveFlag_RXNE(SPI1) != 1U)
    {
        // /* Timeout management */
        // if ((portGetTickCnt() - tickstart) >= DW_SPI_TIMEOUT)
        // {
        //     return 1;
        // }
    }
    LL_SPI_ReceiveData8(SPI1); // Dummy read as we write the header


    /* body */
    for (i = 0; i < len; i++)
    {
        data = is_read ? 0x00 : buf[i];
        LL_SPI_TransmitData8(SPI1, data); // Write data

        while (LL_SPI_IsActiveFlag_RXNE(SPI1) != 1U)
        {
            // /* Timeout management */
            // if ((portGetTickCnt() - tickstart) >= DW_SPI_TIMEOUT)
            // {
            //     return 1;
            // }
        }

        data = LL_SPI_ReceiveData8(SPI1); // this clears RXNE bit
        if (is_read) {
            buf[i] = data;
        }
    }

    LL_GPIO_SetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
    {
        /* Timeout management */
        // if ((portGetTickCnt() - tickstart) >= DW_SPI_TIMEOUT)
        // {
        //     return 1;
        // }
    }

    LL_SPI_Disable(SPI1);
}

void hal_spi_write(u1_t cmd, const u1_t* buf, size_t len) {
    hal_spi_trx(cmd, (u1_t*)buf, len, 0);
}

void hal_spi_read(u1_t cmd, u1_t* buf, size_t len) {
    hal_spi_trx(cmd, buf, len, 1);
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    // Nothing to do
}

u4_t hal_ticks () {
    // Because micros() is scaled down in this function, micros() will
    // overflow before the tick timer should, causing the tick timer to
    // miss a significant part of its values if not corrected. To fix
    // this, the "overflow" serves as an overflow area for the micros()
    // counter. It consists of three parts:
    //  - The US_PER_OSTICK upper bits are effectively an extension for
    //    the micros() counter and are added to the result of this
    //    function.
    //  - The next bit overlaps with the most significant bit of
    //    micros(). This is used to detect micros() overflows.
    //  - The remaining bits are always zero.
    //
    // By comparing the overlapping bit with the corresponding bit in
    // the micros() return value, overflows can be detected and the
    // upper bits are incremented. This is done using some clever
    // bitwise operations, to remove the need for comparisons and a
    // jumps, which should result in efficient code. By avoiding shifts
    // other than by multiples of 8 as much as possible, this is also
    // efficient on AVR (which only has 1-bit shifts).
    // static uint8_t overflow = 0; <-- moved to struct which is preserved after sleep

    // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
    // the others will be the lower bits of our return value.
    uint32_t scaled = getCurrentMicro() >> US_PER_OSTICK_EXPONENT;
    // Most significant byte of scaled
    uint8_t msb = scaled >> 24;
    // Mask pointing to the overlapping bit in msb and overflow.
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    // Update overflow. If the overlapping bit is different
    // between overflow and msb, it is added to the stored value,
    // so the overlapping bit becomes equal again and, if it changed
    // from 1 to 0, the upper bits are incremented.
    hal_vars.overflow += (msb ^ hal_vars.overflow) & mask;

    // Return the scaled value with the upper bits of stored added. The
    // overlapping bit will be equal and the lower bits will be 0, so
    // bitwise or is a no-op for them.
    return scaled | ((uint32_t)hal_vars.overflow << 24);

    // // 0 leads to correct, but overly complex code (it could just return
    // // micros() unmodified), 8 leaves no room for the overlapping bit.
    // static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

// deal with boards that are stressed by no-interrupt delays #529, etc.
#if defined(ARDUINO_DISCO_L072CZ_LRWAN1)
# define HAL_WAITUNTIL_DOWNCOUNT_MS 16      // on this board, 16 ms works better
# define HAL_WAITUNTIL_DOWNCOUNT_THRESH ms2osticks(16)  // as does this threashold.
#else
# define HAL_WAITUNTIL_DOWNCOUNT_MS 8       // on most boards, delay for 8 ms
# define HAL_WAITUNTIL_DOWNCOUNT_THRESH ms2osticks(9) // but try to leave a little slack for final timing.
#endif

u4_t hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    // check for already too late.
    if (delta < 0)
        return -delta;

    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383. Also, STM32 does a better
    // job with delay is less than 10,000 us; so reduce in steps.
    // It's nice to use delay() for the longer times.
    while (delta > HAL_WAITUNTIL_DOWNCOUNT_THRESH) {
        // alternative a: original
        // // deliberately delay 8ms rather than 9ms, so we
        // // will exit loop with delta typically positive.
        // // Depends on BSP keeping time accurately even if interrupts
        // // are disabled.
        // LL_mDelay(HAL_WAITUNTIL_DOWNCOUNT_MS);

        // alternative b: sleep between ticks, wakeup from systick (after 1ms)
        __WFI();
        // re-synchronize.
        delta = delta_time(time);
    }

    // The radio driver runs with interrupt disabled, and this can
    // mess up timing APIs on some platforms. If we know the BSP feature
    // set, we can decide whether to use delta_time() [more exact, 
    // but not always possible with interrupts off], or fall back to
    // delay_microseconds() [less exact, but more universal]

#if defined(_mcci_arduino_version)
    // unluckily, delayMicroseconds() isn't very accurate.
    // but delta_time() works with interrupts disabled.
    // so spin using delta_time().
    while (delta_time(time) > 0)
        /* loop */;
#else // ! defined(_mcci_arduino_version)
    // on other BSPs, we need to stick with the older way,
    // until we fix the radio driver to run with interrupts
    // enabled.
    if (delta > 0)
        delayMicroseconds(delta * US_PER_OSTICK);
#endif // ! defined(_mcci_arduino_version)

    // we aren't "late". Callers are interested in gross delays, not
    // necessarily delays due to poor timekeeping here.
    return 0;
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    NVIC_DisableIRQ(EXTI2_3_IRQn);
    NVIC_DisableIRQ(EXTI4_15_IRQn);

    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        // disable DIO interrupts
        NVIC_SetPriority(EXTI2_3_IRQn, 0);
        NVIC_EnableIRQ(EXTI2_3_IRQn);
        NVIC_SetPriority(EXTI4_15_IRQn, 0);
        NVIC_EnableIRQ(EXTI4_15_IRQn);

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}

uint8_t hal_getIrqLevel(void) {
    return irqlevel;
}

void hal_sleep () {
    // Not implemented
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
#if !defined(__AVR)
static ssize_t uart_putchar (void *, const char *buf, size_t len) {
    return LMIC_PRINTF_TO.write((const uint8_t *)buf, len);
}

static cookie_io_functions_t functions =
 {
     .read = NULL,
     .write = uart_putchar,
     .seek = NULL,
     .close = NULL
 };

void hal_printf_init() {
    stdout = fopencookie(NULL, "w", functions);
    if (stdout != nullptr) {
        setvbuf(stdout, NULL, _IONBF, 0);
    }
}
#else // defined(__AVR)
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init() {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}

#endif // !defined(ESP8266) || defined(ESP31B) || defined(ESP32)
#endif // defined(LMIC_PRINTF_TO)

void hal_init (void) {
    hal_vars.overflow = 0;
}

// hal_init_ex is a C API routine, written in C++, and it's called
// with a pointer to an lmic_pinmap.
void hal_init_ex (const void *pContext) {
    const lmic_pinmap * const pHalPinmap = (const lmic_pinmap *) pContext;
    if (! hal_init_with_pinmap(pHalPinmap)) {
        hal_failed(__FILE__, __LINE__);
    }
}

bool hal_init_with_pinmap(const HalPinmap_t *pPinmap)
    {
    if (pPinmap == NULL)
        return false;

    // set the static pinmap pointer.
    plmic_pins = pPinmap;

    // // set the static HalConfiguration pointer.
    // HalConfiguration_t * const pThisHalConfig = pPinmap->pConfig;

    // if (pThisHalConfig != nullptr)
    //     pHalConfig = pThisHalConfig;
    // else
    //     pHalConfig = &nullHalConig;

    // pHalConfig->begin();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
    // declare success
    return true;
}


void hal_failed (const char *file, u2_t line) {
    if (custom_hal_failure_handler != NULL) {
        (*custom_hal_failure_handler)(file, line);
    }

#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif

    hal_disableIRQs();

    // Infinite loop
    while (1) {
        ;
    }
}

void hal_set_failure_handler(const hal_failure_handler_t* const handler) {
    custom_hal_failure_handler = handler;
}

ostime_t hal_setModuleActive (bit_t val) {
    return 0;
}

bit_t hal_queryUsingTcxo(void) {
    return 0;
}

uint8_t hal_getTxPowerPolicy(
    u1_t inputPolicy,
    s1_t requestedPower,
    u4_t frequency
    ) {
    return LMICHAL_radio_tx_power_policy_paboost;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")
static void delayMicroseconds(uint32_t us)
{
    LL_LPTIM_Enable(LPTIM1);
    LL_LPTIM_SetAutoReload(LPTIM1, 0xFFFF);
    LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);
    
    // wait for timer to finish
    volatile uint32_t timval = 0;
    while (timval < us) {
        timval = LL_LPTIM_GetCounter(LPTIM1);
    }
}
#pragma GCC pop_options