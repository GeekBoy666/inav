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
#include <stdlib.h>

#include "platform.h"

#include "common/utils.h"

#include "gpio.h"

#include "light_led.h"

void ledInit(void)
{
    GPIO_InitTypeDef ioinit;
    ioinit.Alternate = 0xFF;
    ioinit.Mode = GPIO_MODE_OUTPUT_PP;
    ioinit.Speed = GPIO_SPEED_LOW;
    ioinit.Pull = GPIO_NOPULL;
    
#ifdef LED0
    ioinit.Pin = LED0_PIN;
    HAL_GPIO_Init(LED0_GPIO, &ioinit);
#endif
#ifdef LED1
    ioinit.Pin = LED1_PIN;
    HAL_GPIO_Init(LED1_GPIO, &ioinit);
#endif
#ifdef LED2
    ioinit.Pin = LED2_PIN;
    HAL_GPIO_Init(LED2_GPIO, &ioinit);
#endif

    LED0_ON;
    LED1_ON;
    LED2_ON;

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;


}

