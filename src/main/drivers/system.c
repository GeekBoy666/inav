/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "gpio.h"
#include "light_led.h"
#include "sound_beeper.h"
#include "nvic.h"

#include "system.h"

#ifndef EXTI_CALLBACK_HANDLER_COUNT
#define EXTI_CALLBACK_HANDLER_COUNT 1
#endif

extiCallbackHandlerConfig_t extiHandlerConfigs[EXTI_CALLBACK_HANDLER_COUNT];

void registerExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn)
{
    for (int index = 0; index < EXTI_CALLBACK_HANDLER_COUNT; index++) {
        extiCallbackHandlerConfig_t *candidate = &extiHandlerConfigs[index];
        if (!candidate->fn) {
            candidate->fn = fn;
            candidate->irqn = irqn;
            return;
        }
    }
    failureMode(FAILURE_DEVELOPER); // EXTI_CALLBACK_HANDLER_COUNT is too low for the amount of handlers required.
}

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;

void cycleCounterInit(void)
{
#ifdef USE_HAL_DRIVER
    usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
#else
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
#endif
}

// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;
#ifdef USE_HAL_DRIVER
    // used by the HAL for some timekeeping and timeouts, should always be 1ms
    HAL_IncTick();
#endif
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
        /*
         * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
         * interrupt to be delivered before we can recheck sysTickUptime:
         */
        asm volatile("\tnop\n");
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}
#ifdef USE_HAL_DRIVER
void systemInit(void)
{

    

    // Priority grouping should be setup before any irq prio is set.
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);
   
    
    // GPIO ports are always used so it makes sense to just enable the clocks to the available ports
    #ifdef GPIOA
        __HAL_RCC_GPIOA_CLK_ENABLE();
    #endif    
    #ifdef GPIOB
        __HAL_RCC_GPIOB_CLK_ENABLE();
    #endif
    #ifdef GPIOC
        __HAL_RCC_GPIOC_CLK_ENABLE();
    #endif
    #ifdef GPIOD
        __HAL_RCC_GPIOD_CLK_ENABLE();
    #endif
    #ifdef GPIOE
        __HAL_RCC_GPIOE_CLK_ENABLE();
    #endif
    #ifdef GPIOF
        __HAL_RCC_GPIOF_CLK_ENABLE();
    #endif
        
    // It makes it all lot easier if we just enable to DMA clocks, the additional idle current is neglectable
    #ifdef DMA1
        __HAL_RCC_DMA1_CLK_ENABLE();
    #endif
    #ifdef DMA2
        __HAL_RCC_DMA2_CLK_ENABLE();
    #endif
    
    /* Select SysClk as source of USART1 clocks */
    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    RCC_PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
        
    // Init clocks of used peripherals        
    #ifdef USE_USART1
    __HAL_RCC_USART1_CLK_ENABLE();
    #endif
    #ifdef USE_USART2
    __HAL_RCC_USART2_CLK_ENABLE();
    #endif
    #ifdef USE_USART3
    __HAL_RCC_USART3_CLK_ENABLE();
    #endif
    #ifdef USE_USART4
    __HAL_RCC_UART4_CLK_ENABLE();
    #endif
    #ifdef USE_USART5
    __HAL_RCC_UART5_CLK_ENABLE();
    #endif
    #ifdef USE_USART6
    __HAL_RCC_USART6_CLK_ENABLE();
    #endif
    #ifdef USE_USART7
    __HAL_RCC_UART7_CLK_ENABLE();
    #endif
    #ifdef USE_USART8
    __HAL_RCC_UART8_CLK_ENABLE();
    #endif
    

}
#endif

#if 1
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}
#else
void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;

    for (;;) {
        register uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
#endif

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

#define SHORT_FLASH_DURATION 50
#define CODE_FLASH_DURATION 250

void failureMode(failureMode_e mode)
{
    int codeRepeatsRemaining = 10;
    int codeFlashesRemaining;
    int shortFlashesRemaining;

    while (codeRepeatsRemaining--) {
        LED1_ON;
        LED0_OFF;
        shortFlashesRemaining = 5;
        codeFlashesRemaining = mode + 1;
        uint8_t flashDuration = SHORT_FLASH_DURATION;

        while (shortFlashesRemaining || codeFlashesRemaining) {
            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_ON;
            delay(flashDuration);

            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_OFF;
            delay(flashDuration);

            if (shortFlashesRemaining) {
                shortFlashesRemaining--;
                if (shortFlashesRemaining == 0) {
                    delay(500);
                    flashDuration = CODE_FLASH_DURATION;
                }
            } else {
                codeFlashesRemaining--;
            }
        }
        delay(1000);
    }

#ifdef DEBUG
    systemReset();
#else
    systemResetToBootloader();
#endif
}
