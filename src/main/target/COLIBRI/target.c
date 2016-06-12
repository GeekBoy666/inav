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
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),		// USART2
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),     // USART2 // Swap to servo if needed
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), // motor #2
    PWM16 | (MAP_TO_SERVO_OUTPUT << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8), // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT << 8), // servo #3
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8), // servo #4
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),    // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),    // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), // motor #2
    PWM16 | (MAP_TO_SERVO_OUTPUT << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8), // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT << 8), // servo #3
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8), // servo #4
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource10, GPIO_AF_TIM1},    // S1_IN
    { TIM8, GPIOC, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource6, GPIO_AF_TIM8}, // S2_IN
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM8}, // S3_IN
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM8}, // S4_IN
    { TIM2, GPIOA, Pin_15, TIM_Channel_1, TIM2_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM2},   // S5_IN
    { TIM2, GPIOB, Pin_3, TIM_Channel_2, TIM2_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM2},   // S6_IN
    { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5},   // S7_IN
    { TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5},   // S8_IN


    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},    // S1_OUT
    { TIM3, GPIOB, Pin_4, TIM_Channel_1, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource4, GPIO_AF_TIM3},    // S2_OUT
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},    // S3_OUT
    { TIM12, GPIOB, Pin_15, TIM_Channel_2, TIM8_BRK_TIM12_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM12}, // S4_OUT
    { TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource5, GPIO_AF_TIM3},    // S5_OUT
    { TIM12, GPIOB, Pin_14, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM12}, // S6_OUT
    { TIM10, GPIOB, Pin_8, TIM_Channel_1, TIM1_UP_TIM10_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM10},    // S7_OUT
    { TIM11, GPIOB, Pin_9, TIM_Channel_1, TIM1_TRG_COM_TIM11_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM11},    // S8_OUT
};
