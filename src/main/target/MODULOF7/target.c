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

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2 or servo #2 (swap to servo if needed)
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1 or #3
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #3
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

//   Timer, port , pin  , Channel      ,  IRQn       , out, mode,   , pinsource      , alternate function
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // inputs
    { TIM1  , IO_TAG(PE9)   , TIM_CHANNEL_1, TIM1_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF1_TIM1},               // RX1,         PE9
    { TIM1  , IO_TAG(PE11)  , TIM_CHANNEL_2, TIM1_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF1_TIM1},             // RX2,         PE11
    { TIM1  , IO_TAG(PE13)  , TIM_CHANNEL_3, TIM1_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF1_TIM1},             // RX3,         PE13
    { TIM1  , IO_TAG(PE14)  , TIM_CHANNEL_4, TIM1_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF1_TIM1},             // RX4,         PE14
    { TIM3  , IO_TAG(PE5)   , TIM_CHANNEL_3, TIM3_IRQn, 0, IOCFG_AF_PP, GPIO_AF2_TIM3},                  // RX5          PE5
    { TIM3  , IO_TAG(PE6)   , TIM_CHANNEL_4, TIM3_IRQn, 0, IOCFG_AF_PP, GPIO_AF2_TIM3},                  // RX6,         PE6
    
    // outputs
    { TIM4  , IO_TAG(PD12)  , TIM_CHANNEL_1, TIM4_IRQn, 1, IOCFG_AF_PP, GPIO_AF2_TIM4},                // OUT1,        PD12
    { TIM4  , IO_TAG(PD13)  , TIM_CHANNEL_2, TIM4_IRQn, 1, IOCFG_AF_PP, GPIO_AF2_TIM4},                // OUT2,        PD13
    { TIM4  , IO_TAG(PD14)  , TIM_CHANNEL_3, TIM4_IRQn, 1, IOCFG_AF_PP, GPIO_AF2_TIM4},                // OUT3,        PD14
    { TIM4  , IO_TAG(PD15)  , TIM_CHANNEL_4, TIM4_IRQn, 1, IOCFG_AF_PP, GPIO_AF2_TIM4},                // OUT4,        PD15
            
    { TIM9  , IO_TAG(PE5)   , TIM_CHANNEL_1, TIM1_BRK_TIM9_IRQn, 1, IOCFG_AF_PP, GPIO_AF3_TIM9},         // OUT5,        PE5
    { TIM9  , IO_TAG(PE6)   , TIM_CHANNEL_2, TIM1_BRK_TIM9_IRQn, 1, IOCFG_AF_PP, GPIO_AF3_TIM9},         // OUT6,        PE6
    
    { TIM10 , IO_TAG(PB8)   , TIM_CHANNEL_1, TIM1_UP_TIM10_IRQn, 1, IOCFG_AF_PP, GPIO_AF3_TIM10},       // OUT7,        PB8

    { TIM11 , IO_TAG(PB9)   , TIM_CHANNEL_1, TIM1_TRG_COM_TIM11_IRQn, 1, IOCFG_AF_PP, GPIO_AF3_TIM11},  // OUT8,        PB9
    
    // other
    { TIM13 , IO_TAG(PA6)   , TIM_CHANNEL_1, TIM8_UP_TIM13_IRQn, 1, IOCFG_AF_PP, GPIO_AF9_TIM13},       // LEDStrip,    PA6
                            
    { TIM14 , IO_TAG(PA7)   , TIM_CHANNEL_1, TIM8_TRG_COM_TIM14_IRQn, 1, IOCFG_AF_PP, GPIO_AF9_TIM14},  // IR,          PA7
    
    // optional extra but shared with other default functions
    { TIM5  , IO_TAG(PA0)   , TIM_CHANNEL_1, TIM5_IRQn, 0, IOCFG_AF_PP, GPIO_AF9_TIM13},                 // UART4_TX,    PA0
    { TIM5  , IO_TAG(PA1)   , TIM_CHANNEL_2, TIM5_IRQn, 0, IOCFG_AF_PP, GPIO_AF9_TIM13},                 // UART4_RX,    PA1
    
    { TIM2  , IO_TAG(PA5)   , TIM_CHANNEL_1, TIM2_IRQn, 0, IOCFG_AF_PP, GPIO_AF9_TIM13},                 // ANALOG_DAC,  PA5
    { TIM2  , IO_TAG(PB10)  , TIM_CHANNEL_3, TIM2_IRQn, 0, IOCFG_AF_PP, GPIO_AF9_TIM13},               // I2C2_SCL,    PB10
    { TIM2  , IO_TAG(PB11)  , TIM_CHANNEL_4, TIM2_IRQn, 0, IOCFG_AF_PP, GPIO_AF9_TIM13},               // I2C2_SDA,    PB11
    
    { TIM8  , IO_TAG(PC6)   , TIM_CHANNEL_1, TIM8_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF9_TIM13},              // UART6_TX,    PC6
    { TIM8  , IO_TAG(PC7)   , TIM_CHANNEL_2, TIM8_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF9_TIM13},              // UART6_RX,    PC7
    { TIM8  , IO_TAG(PC9)   , TIM_CHANNEL_4, TIM8_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF9_TIM13},              // I2C3_SDA,    PC9
};
