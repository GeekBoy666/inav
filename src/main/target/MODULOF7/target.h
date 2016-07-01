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

#pragma once

#include "../portable.h"

#define TARGET_BOARD_IDENTIFIER "MODULOF7"

//#define USE_QUAD_MIXER_ONLY

#ifndef USE_HAL_DRIVER
#define USE_HAL_DRIVER
#endif

#ifndef FLASH_SIZE
#define FLASH_SIZE (1024)
#endif
#define FLASH_PAGE_COUNT FLASH_SECTOR_TOTAL
#define FLASH_PAGE_SIZE ((FLASH_SIZE*1024)/FLASH_SECTOR_TOTAL)

#define LED0_GPIO   GPIOE
#define LED0_PIN    Pin_15 // LED1 PE15
#define LED1_GPIO   GPIOE
#define LED1_PIN    Pin_12  // LED2 PE12

//#define INVERTER_PIN Pin_0 // PC0 used as inverter select GPIO
//#define INVERTER_GPIO GPIOC
//#define INVERTER_PERIPHERAL RCC_AHB1Periph_GPIOC
//#define INVERTER_USART USART1

#define MPU6000_CS_GPIO       GPIOA
#define MPU6000_CS_PIN        Pin_15
#define MPU6000_SPI_INSTANCE  SPIDEV_1

//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL

#define ACC
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN CW270_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN CW270_DEG

//#define MAG
////#define USE_MAG_HMC5883
//#define HMC5883_BUS I2C_DEVICE_INT
//#define MAG_HMC5883_ALIGN CW90_DEG

#define BARO
#define USE_BARO_BMP280
#define BMP280_BUS I2C_DEVICE_INT

//#define PITOT
//#define USE_PITOT_MS4525
//#define MS4525_BUS I2C_DEVICE_EXT

#define I2CGPS_BUS I2C_DEVICE_INT

//#define INVERTER
#define LED0
#define LED1

//#define M25P16_CS_GPIO        GPIOB
//#define M25P16_CS_PIN         GPIO_Pin_3
//#define M25P16_SPI_INSTANCE   SPI3

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

#define USABLE_TIMER_CHANNEL_COUNT 24

#define USE_VCP

#define USE_USART1
#define USART1_RX_PIN Pin_10
#define USART1_TX_PIN Pin_9
#define USART1_GPIO GPIOA
#define USE_USART1_RX_DMA false
#define USE_USART1_TX_DMA true


//#define USE_USART2
//#define USART2_RX_PIN Pin_7
//#define USART2_TX_PIN Pin_6
//#define USART2_GPIO GPIOC

//#define USE_USART3
//#define USART3_RX_PIN Pin_11
//#define USART3_TX_PIN Pin_10
//#define USART3_GPIO GPIOB

//#define USE_USART4
//#define USART4_RX_PIN Pin_7
//#define USART4_TX_PIN Pin_6
//#define USART4_GPIO GPIOC

//#define USE_USART6
//#define USART6_RX_PIN Pin_7
//#define USART6_TX_PIN Pin_6
//#define USART6_GPIO GPIOC

#define SERIAL_PORT_COUNT (6) // 5 x U(S)ART 1x USB vcp
#define UART_INDEX_MAX (8)

//#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_GPIO   GPIOB
#define SPI1_SCK_PIN    Pin_3
#define SPI1_NSS_GPIO   GPIOA
#define SPI1_NSS_PIN    Pin_15
#define SPI1_MISO_GPIO  GPIOB
#define SPI1_MISO_PIN   Pin_4
#define SPI1_MOSI_GPIO  GPIOB
#define SPI1_MOSI_PIN   Pin_5

#define USE_SPI_DEVICE_1
#define SPI2_SCK_GPIO   GPIOB
#define SPI2_SCK_PIN    Pin_13
#define SPI2_NSS_GPIO   GPIOB
#define SPI2_NSS_PIN    Pin_12
#define SPI2_MISO_GPIO  GPIOB
#define SPI2_MISO_PIN   Pin_14
#define SPI2_MOSI_GPIO  GPIOB
#define SPI2_MOSI_PIN   Pin_15

#define USE_SPI_DEVICE_1
#define SPI3_SCK_GPIO   GPIOC
#define SPI3_SCK_PIN    Pin_10
#define SPI3_NSS_GPIO   GPIOA
#define SPI3_NSS_PIN    Pin_4
#define SPI3_MISO_GPIO  GPIOC
#define SPI3_MISO_PIN   Pin_11
#define SPI3_MOSI_GPIO  GPIOC
#define SPI3_MOSI_PIN   Pin_12

//#define USE_SPI_DEVICE_3

#define USE_I2C
#define I2C_DEVICE_INT (I2CDEV_1)
#define I2C_DEVICE_EXT (I2CDEV_2)

#define I2C1_SCL_GPIO GPIOB
#define I2C1_SCL_PIN Pin_6
#define I2C1_SDA_GPIO GPIOB
#define I2C1_SDA_PIN Pin_7

#define I2C2_SCL_GPIO GPIOB
#define I2C2_SCL_PIN Pin_10
#define I2C2_SDA_GPIO GPIOB
#define I2C2_SDA_PIN Pin_11

#define I2C3_SCL_GPIO GPIOA
#define I2C3_SCL_PIN Pin_8
#define I2C3_SDA_GPIO GPIOC
#define I2C3_SDA_PIN Pin_9


#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

//#define LED_STRIP
//#define LED_STRIP_TIMER TIM5

#define GPS
#define GPS_PROTO_NMEA
#define GPS_PROTO_UBLOX
#define GPS_PROTO_UBLOX_NEO7PLUS
#define GPS_PROTO_I2C_NAV
#define GPS_PROTO_NAZA
#define MAG_GPS_ALIGN CW180_DEG_FLIP

#define NAV
#define NAV_AUTO_MAG_DECLINATION
#define NAV_GPS_GLITCH_DETECTION

#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define GTUNE
#define USE_SERVOS
#define USE_CLI
#define UG2864_BUS I2C_DEVICE_EXT
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11) | TIM_N(12) | TIM_N(13) | TIM_N(14))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_TIM12 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9)
