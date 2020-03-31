/**
  ******************************************************************************
  * File Name          : eeprom.c
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
#include "eeprom.h"

// save LMIC and tick which is to be restored after standby
uint8_t eeprom_save(uint32_t tick_save)
{
    uint32_t primask_bit;
    struct eeprom_s * eeprom_ptr;
    uint8_t oldest_instance, newest_instance;

    // unlock eeprom
    if((FLASH->PECR & FLASH_PECR_PELOCK) != RESET)
    {  
        /* Disable interrupts to avoid any interruption during unlock sequence */
        primask_bit = __get_PRIMASK();
        __disable_irq();

        /* Unlocking the Data memory and FLASH_PECR register access*/
        FLASH->PEKEYR = FLASH_PEKEY1;
        FLASH->PEKEYR = FLASH_PEKEY2;

        /* Re-enable the interrupts: restore previous priority mask */
        __set_PRIMASK(primask_bit);

        if((FLASH->PECR & FLASH_PECR_PELOCK) != RESET)
        {
            return 1;
        }
    }

    // set pointer to beginning of EEPROM
    eeprom_ptr = (struct eeprom_s *) DATA_E2_ADDR;

    // wear leveling: find oldest data set (smallest valid counter) to be overwritten
    oldest_instance = 0;
    newest_instance = 0;
    for (int i = 0; i < DATA_E2_NUM_INSTANCES; i++)
    {
        oldest_instance = (eeprom_ptr[i].valid < eeprom_ptr[oldest_instance].valid) ? i : oldest_instance;
        newest_instance = (eeprom_ptr[i].valid > eeprom_ptr[newest_instance].valid) ? i : newest_instance;
    }

    // copy data
    eeprom_ptr[oldest_instance].valid = eeprom_ptr[newest_instance].valid + 1;  // set to new current dataset
    eeprom_ptr[oldest_instance].lmic_instance = LMIC;   // LMIC is global variable
    eeprom_ptr[oldest_instance].tick = tick_save;
    eeprom_ptr[oldest_instance].hal_instance = hal_get_persistence();

    /* Set the PELOCK Bit to lock the data memory and FLASH_PECR register access */
    SET_BIT(FLASH->PECR, FLASH_PECR_PELOCK);

    return 0;
}

// restore LMIC and setTick()
uint8_t eeprom_restore(void)
{
    uint32_t primask_bit;
    struct eeprom_s * eeprom_ptr;
    uint8_t oldest_instance, newest_instance;

    // set pointer to beginning of EEPROM
    eeprom_ptr = (struct eeprom_s *) DATA_E2_ADDR;

    // wear leveling: find oldest data set (smallest valid counter) to be overwritten
    oldest_instance = 0;
    newest_instance = 0;
    for (int i = 0; i < DATA_E2_NUM_INSTANCES; i++)
    {
        oldest_instance = (eeprom_ptr[i].valid < eeprom_ptr[oldest_instance].valid) ? i : oldest_instance;
        newest_instance = (eeprom_ptr[i].valid > eeprom_ptr[newest_instance].valid) ? i : newest_instance;
    }

    /* disable irq to not get interrupted at systick restore */
    primask_bit = __get_PRIMASK();
    __disable_irq();

    // copy data
    LMIC = eeprom_ptr[newest_instance].lmic_instance;   // LMIC is global variable
    setTick(eeprom_ptr[newest_instance].tick);
    hal_set_persistence(eeprom_ptr[newest_instance].hal_instance);

    /* Re-enable the interrupts: restore previous priority mask */
    __set_PRIMASK(primask_bit);

    return 0;
}

// clear all eeprom data
uint8_t eeprom_clear(void)
{
    uint32_t primask_bit;
    struct eeprom_s * eeprom_ptr;

    // unlock eeprom
    if((FLASH->PECR & FLASH_PECR_PELOCK) != RESET)
    {  
        /* Disable interrupts to avoid any interruption during unlock sequence */
        primask_bit = __get_PRIMASK();
        __disable_irq();

        /* Unlocking the Data memory and FLASH_PECR register access*/
        FLASH->PEKEYR = FLASH_PEKEY1;
        FLASH->PEKEYR = FLASH_PEKEY2;

        /* Re-enable the interrupts: restore previous priority mask */
        __set_PRIMASK(primask_bit);

        if((FLASH->PECR & FLASH_PECR_PELOCK) != RESET)
        {
            return 1;
        }
    }

    // set address and zero out all valid counters
    eeprom_ptr = (struct eeprom_s *) DATA_E2_ADDR;

    for (int i = 0; i < DATA_E2_NUM_INSTANCES; i++)
    {
        eeprom_ptr[i].valid = 0;
    }
    
    /* Set the PELOCK Bit to lock the data memory and FLASH_PECR register access */
    SET_BIT(FLASH->PECR, FLASH_PECR_PELOCK);

    return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
