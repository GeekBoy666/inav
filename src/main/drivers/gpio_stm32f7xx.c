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

#include "build/build_config.h"

#include "gpio.h"

#define MODE_OFFSET 0
#define PUPD_OFFSET 8

#define MODE_MASK (0xFF)
#define PUPD_MASK (0xFF)


//#define GPIO_Speed_10MHz GPIO_Speed_Level_1   Fast Speed:10MHz
//#define GPIO_Speed_2MHz  GPIO_Speed_Level_2   Medium Speed:2MHz
//#define GPIO_Speed_50MHz GPIO_Speed_Level_3   High Speed:50MHz

void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    uint32_t pinIndex;
    for (pinIndex = 0; pinIndex < 16; pinIndex++) {
        // are we doing this pin?
        uint32_t pinMask = (0x1 << pinIndex);
        if (config->pin & pinMask) {

            GPIO_InitStructure.Pin =  pinMask;
            GPIO_InitStructure.Mode = config->mode;

            uint32_t speed = GPIO_SPEED_FREQ_MEDIUM;
            switch (config->speed) {
                case Speed_10MHz:
                    speed = GPIO_SPEED_FREQ_MEDIUM;
                    break;
                case Speed_2MHz:
                    speed = GPIO_SPEED_FREQ_LOW;
                    break;
                case Speed_50MHz:
                    speed = GPIO_SPEED_FREQ_HIGH;
                    break;
            }

            GPIO_InitStructure.Speed = speed;
//            GPIO_InitStructure.GPIO_OType = (config->mode >> OUTPUT_OFFSET) & OUTPUT_MASK;
            GPIO_InitStructure.Pull = GPIO_NOPULL;
            if((config->mode == Mode_AF_PP_PU) || config->mode == Mode_IPU)
            {
                GPIO_InitStructure.Pull = GPIO_PULLUP;
            } 
            else if((config->mode == Mode_AF_PP_PD) || config->mode == Mode_IPD)
            {
                GPIO_InitStructure.Pull = GPIO_PULLDOWN;
            }
            
            GPIO_InitStructure.Alternate = config->AltFunction;
            HAL_GPIO_Init(gpio, &GPIO_InitStructure);
        }
    }
}

void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc)
{
    UNUSED(portsrc);
    UNUSED(pinsrc);
    // FIXME needed yet? implement?
}
