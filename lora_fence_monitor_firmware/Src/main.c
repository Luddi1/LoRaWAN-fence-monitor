/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "comp.h"
#include "dma.h"
#include "lptim.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arduino_lmic.h"
#include "hal/hal.h"
#include "fence.h"
#include "eeprom.h"

#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* important!! changes after code generation in cubemx
 * gpio.c:53 
 *  // if called resets radio -> standby mode -> 1.6mA -> not good 
 *  // LL_GPIO_ResetOutputPin(RFM95_RST_GPIO_Port, RFM95_RST_Pin); 
 * 
 * main.c
 *  // LL_RCC_ForceBackupDomainReset();
 *  // LL_RCC_ReleaseBackupDomainReset();
 * 
 */

/*  Lora message structure, 1 byte long
 *  Byte 0: 
 *      bit 7:      Timeout
 *      bit 6-0:    Fencevoltage / 0.1kV  (e.g. 11.4kV = 114 in message, max 12.7kV)
 */
#define LORA_MESSAGE_NUM_BYTES      1

#define LORA_MESSAGE_BATTERYLOW_BYTE   (0)
#define LORA_MESSAGE_BATTERYLOW_MASK   (1<<7)
#define LORA_MESSAGE_VOLTAGE_BYTE   (0)
#define LORA_MESSAGE_VOLTAGE_MASK   (0x7F)

#define PRINTBUF_SIZE   (40)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void EnterStandbyMode(void);
void EnterSTOPMode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t APPEUI[8]={ 0x1D, 0x98, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t DEVEUI[8]={ 0xE6, 0xEB, 0xA6, 0x07, 0xCD, 0x00, 0xF9, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t APPKEY[16] = { 0xF8, 0x02, 0x16, 0xAC, 0xF7, 0xE7, 0xD9, 0xA0, 0xC6, 0xDA, 0x65, 0x86, 0x00, 0x16, 0xBE, 0x09 };
void os_getDevKey (u1_t* buf) {  memcpy(buf, APPKEY, 16);}

static char printbuf[PRINTBUF_SIZE];

static uint8_t loradata[LORA_MESSAGE_NUM_BYTES] = {0};
static uint8_t lora_port = 2;
static uint8_t lora_confirmed = 0;
static osjob_t sendjob;

// Schedule measurement every this many seconds
static uint16_t check_interval = 5*60;      // default 5 min
static uint16_t heartbeat_counter = 24;      // (24) for every 2h at 5min check interval

static uint16_t retry_interval = 10;         // check for no pending jobs next secs to go to sleep

static volatile uint8_t txcomplete = 0;     // set, when tx done and ready for next sleep
static uint32_t fence_timeout = 3000;       // timeout value for fence in ms

// Pin mapping, unused in this implementation
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

void do_send(osjob_t* j)
{
    lmic_tx_error_t error = 0;

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        MYPRINT("OP_TXRXPEND, not sending\r\n");
    } else {
        MYPRINT("Packet queued\r\n");
        // Prepare upstream data transmission at the next possible time.
        error = LMIC_setTxData2(lora_port, loradata, LORA_MESSAGE_NUM_BYTES, lora_confirmed);
    }

    switch (error)
    {
    case LMIC_ERROR_TX_BUSY:
        MYPRINT("LMIC_ERROR_TX_BUSY\r\n");
        break;
    case LMIC_ERROR_TX_TOO_LARGE:
        MYPRINT("LMIC_ERROR_TX_TOO_LARGE\r\n");
        break;
    case LMIC_ERROR_TX_NOT_FEASIBLE:
        MYPRINT("LMIC_ERROR_TX_NOT_FEASIBLE\r\n");
        break;
    case LMIC_ERROR_TX_FAILED:
        MYPRINT("LMIC_ERROR_TX_FAILED\r\n");
        break;
    default:
        break;
    }
}

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            MYPRINT("EV_SCAN_TIMEOUT\r\n");
            break;
        case EV_BEACON_FOUND:
            MYPRINT("EV_BEACON_FOUND\r\n");
            break;
        case EV_BEACON_MISSED:
            MYPRINT("EV_BEACON_MISSED\r\n");
            break;
        case EV_BEACON_TRACKED:
            MYPRINT("EV_BEACON_TRACKED\r\n");
            break;
        case EV_JOINING:
            // normally starts at DR_SF7 and increases if not heard
            // /* set SF10 to get reliable join, potentially decreased over time by ADR */
            // LMIC_setDrTxpow(DR_SF10,14);
            MYPRINT("EV_JOINING\r\n");
            break;
        case EV_JOINED:
            MYPRINT("EV_JOINED\r\n");
            /* enable ADR */
            LMIC_setAdrMode(1);
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            MYPRINT("EV_JOIN_FAILED\r\n");
            break;
        case EV_REJOIN_FAILED:
            MYPRINT("EV_REJOIN_FAILED\r\n");
            break;
        case EV_TXCOMPLETE:
            MYPRINT("EV_TXCOMPLETE\r\n");
            if (LMIC.txrxFlags & TXRX_ACK) {
              MYPRINT("Received ack\r\n");
            }
            if (LMIC.dataLen) {
              MYPRINT("Received data\r\n");
            }
            txcomplete = 1;
            break;
        case EV_JOIN_TXCOMPLETE:
            // join accept not received
            MYPRINT("EV_JOIN_TXCOMPLETE\r\n");
            break;
        case EV_LOST_TSYNC:
            MYPRINT("EV_LOST_TSYNC\r\n");
            break;
        case EV_RESET:
            MYPRINT("EV_RESET\r\n");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            MYPRINT("EV_RXCOMPLETE\r\n");
            break;
        case EV_LINK_DEAD:
            MYPRINT("EV_LINK_DEAD\r\n");
            break;
        case EV_LINK_ALIVE:
            MYPRINT("EV_LINK_ALIVE\r\n");
            break;
        case EV_TXSTART:
            MYPRINT("EV_TXSTART\r\n");
            break;
        default:
            MYPRINT("Unknown event\r\n");
            break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t from_standby = 0;
    uint32_t fence_start_tick;
    uint8_t fence_timeout_triggered;    // set to non zero, if timed out
    uint8_t heartbeat_request = 0;
    uint8_t battery_low_flag;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /** PVD Configuration 
  */
  LL_PWR_SetPVDLevel(LL_PWR_PVDLEVEL_2);
  /** Enable the PVD Output 
  */
  LL_PWR_EnablePVD();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Config(16000);    // 16e3 ticks for 1ms

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_COMP1_Init();
  MX_SPI1_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */

#ifdef APP_DEBUG 
    MX_USART1_UART_Init();
#endif // APP_DEBUG

    MX_RTC_Init();

    if (LL_PWR_IsActiveFlag_SB() != 1)
    {
        /* ##### Run after normal reset ##### */
        from_standby = 0;

        MYPRINT("Run after reset\r\n");

        if (!DEBUG_GET_LMIC_FROM_EEPROM)
        {
            MYPRINT("Clear EEPROM and RTC register\r\n");
            eeprom_clear();

            LL_PWR_EnableBkUpAccess();
            LL_RCC_ForceBackupDomainReset();
            LL_RCC_ReleaseBackupDomainReset();
        }
    }   
    else
    {
        /* ##### Run after standby mode ##### */
        from_standby = 1;

        /* Clear Standby flag*/
        LL_PWR_ClearFlag_SB();
        /* Clear wakeup flag*/
        LL_PWR_ClearFlag_WU();
        
        /* Reset RTC Internal Wake up flag */
        LL_RTC_ClearFlag_WUT(RTC); 

        MYPRINT("Run after standby\r\n");
    }

    /* Configure RTC to use WUT = check_interval */
    Configure_RTC(check_interval);
    MYPRINT("Config RTC\r\n");
    
    // start comp+adc for measurement
    MYPRINT("Starting measurement\r\n");
    fence_start();

    fence_start_tick = getTick();
    fence_timeout_triggered = 0;
    while (!fence_done())
    {
        // wait here, TODO sleep
        if ((getTick() - fence_start_tick) > fence_timeout)
        {
            fence_timeout_triggered = 1;

            MYPRINT("Timeout\r\n");
            break;
        }

        // sleep between ticks, wakeup from systick (after 1ms)
        __WFI();
    }

    adc_disable();  // save power

    snprintf(printbuf, PRINTBUF_SIZE, "Voltage:%ld\r\n", fence_get());
    MYPRINT(printbuf);

    // get heartbeat counter and schedule transmission
    // also clears rtc_alarm_reported_flag, so a pending error gets reported 
    // again after heartbeat_counter * wakeup time
    if ( (heartbeat_request = rtc_heartbeat_get(heartbeat_counter)) != 0 )
    {
        MYPRINT("Heartbeat scheduled\r\n");
    }

    // get battery status
    // threshold has to be set high enough for node to be able to
    // still send out during normal operation
    battery_low_flag = (LL_PWR_IsActiveFlag_PVDO()) ? 1 : 0;
    if (battery_low_flag)
    {
        MYPRINT("Battery low\r\n");
    }

    // ### only go on with LMIC if fence measurement has to be sent out ####
    if ( (fence_need_report() ||
         fence_timeout_triggered ||
         heartbeat_request || 
         !from_standby) &&
         !rtc_alarm_reported_flag_get() )   // dont send if already reported
    {
        // build lorawan message
        int32_t voltage = fence_get();

        voltage = (voltage == -1) ? 0 : voltage / 100;      // on error report 0
        voltage = (voltage > 127) ? 127 : voltage;          // clip at 12.7kV
        loradata[LORA_MESSAGE_VOLTAGE_BYTE] |= ((uint32_t)voltage) & LORA_MESSAGE_VOLTAGE_MASK;

        if (battery_low_flag)
        {
            loradata[LORA_MESSAGE_BATTERYLOW_BYTE] |= LORA_MESSAGE_BATTERYLOW_MASK;
        }

        if (!heartbeat_request)
        {
            rtc_alarm_reported_flag_set();
        }

        MYPRINT("Need report\r\n");
    }
    else
    {
        if (rtc_alarm_reported_flag_get())
        {
            MYPRINT("Pending alarm already reported\r\n");
        }

        MYPRINT("No report, standby\r\n");

        // manually reset radio and put to sleep
        hal_pin_rst(0); // drive RST pin low
        hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us
        hal_pin_rst(2); // configure RST pin floating!
        hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms
        os_radio(RADIO_RST);

        /* Enable wake-up timer and enter in standby mode */
        EnterStandbyMode();

        // never reaches here
    }


    MYPRINT("Start LMIC\r\n");

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // settings
    LMIC_setClockError(MAX_CLOCK_ERROR * 4 / 1000);

    /* Check and handle if the system was resumed from StandBy mode 
       restore lmic from eeprom if requested for debugging         */ 
    if(from_standby || DEBUG_GET_LMIC_FROM_EEPROM)
    {
        MYPRINT("Restore session from EEPROM\r\n");

        // get session + tick from eeprom, if valid
        eeprom_restore();
    }
    else
    {
        // sendjob starts joining automatically
    }

    txcomplete = 0;
    do_send(&sendjob);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    os_runloop_once();
    
    /* when no jobs pending and not joining, go to standby */
    if (!os_queryTimeCriticalJobs(sec2osticks(retry_interval)) &&
        !(LMIC.opmode & OP_JOINING) )
    {
        if (txcomplete)
        {
            MYPRINT("Save session and standby\r\n");

            // save session and ticks+sleeptime
            eeprom_save(getTick() + 1000 * check_interval);

            /* Enable wake-up timer and enter in standby mode */
            EnterStandbyMode();
        }
        else
        {
            // TODO: timeout with txcomplete check
        }
    }

    // sleep between ticks, wakeup from systick (after 1ms)
    __WFI();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {
    
  }
  LL_PWR_EnableBkUpAccess();
//   LL_RCC_ForceBackupDomainReset();
//   LL_RCC_ReleaseBackupDomainReset();
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  LL_RCC_EnableRTC();
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }

  LL_Init1msTick(16000000);

  LL_SetSystemCoreClock(16000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Function to configure and enter in STANDBY Mode.
  * @param  None
  * @retval None
  */
void EnterStandbyMode(void)
{
  /* ######## ENABLE WUT #################################################*/
  /* Disable RTC registers write protection */
  LL_RTC_DisableWriteProtection(RTC);
 
  /* Enable wake up counter and wake up interrupt */
  /* Note: Periodic wakeup interrupt should be enabled to exit the device 
     from low-power modes.*/
  LL_RTC_EnableIT_WUT(RTC);
  LL_RTC_WAKEUP_Enable(RTC);
  
  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);
  
  /* ######## ENTER IN STANDBY MODE ######################################*/
  /** Request to enter STANDBY mode
    * Following procedure describe in STM32L0xx Reference Manual
    * See PWR part, section Low-power modes, Standby mode
    */
  /* Reset Internal Wake up flag */
  LL_RTC_ClearFlag_WUT(RTC); 
  
  /* Set Stand-by mode */
  LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
  
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  LL_LPM_EnableDeepSleep();
  
  /* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM)
  __force_stores();
#endif

  /* Request Wait For Interrupt */
  __WFI();
}

void incTick(void)
{
    tick++;
}

void setTick(uint32_t new_tick)
{
    tick = new_tick;
}

uint32_t getTick(void)
{
    return tick;
}

/**
  * @brief  Function called to read the current micro second
  * @param  None
  * @retval None
  */
uint32_t getCurrentMicro(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = getTick();
  uint32_t u = SysTick->LOAD - SysTick->VAL;
  if(LL_SYSTICK_IsActiveCounterFlag()) {
    m = getTick();
    u = SysTick->LOAD - SysTick->VAL;
  }
  return ( m * 1000 + (u * 1000) / SysTick->LOAD);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
