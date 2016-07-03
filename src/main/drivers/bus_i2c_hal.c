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

#include "build_config.h"

#include "gpio.h"
#include "system.h"

#include "bus_i2c.h"
#include "nvic.h"

#ifndef SOFT_I2C

static void i2cUnstick(I2CDevice bus);

typedef struct i2cDevice_t {
    I2C_TypeDef *dev;
    uint8_t GpioAf;
    GPIO_TypeDef *gpioscl;
    uint16_t scl;
    GPIO_TypeDef *gpiosda;
    uint16_t sda;
    uint8_t ev_irq;
    uint8_t er_irq;
    uint32_t clk;
    uint32_t clk_src;
} i2cDevice_t;

static const i2cDevice_t i2cHardwareMap[] = {
    { I2C1, MCU_I2C1_AF ,I2C1_SCL_GPIO, I2C1_SCL_PIN, I2C1_SDA_GPIO, I2C1_SDA_PIN, I2C1_EV_IRQn, I2C1_ER_IRQn, RCC_PERIPHCLK_I2C1, RCC_I2C1CLKSOURCE_SYSCLK },
    { I2C2, MCU_I2C2_AF ,I2C2_SCL_GPIO, I2C2_SCL_PIN, I2C2_SDA_GPIO, I2C2_SDA_PIN, I2C2_EV_IRQn, I2C2_ER_IRQn, RCC_PERIPHCLK_I2C2, RCC_I2C2CLKSOURCE_SYSCLK },
    { I2C3, MCU_I2C3_AF ,I2C3_SCL_GPIO, I2C3_SCL_PIN, I2C3_SDA_GPIO, I2C3_SDA_PIN, I2C3_EV_IRQn, I2C3_ER_IRQn, RCC_PERIPHCLK_I2C3, RCC_I2C3CLKSOURCE_SYSCLK },
};

typedef struct{
    I2C_HandleTypeDef Handle;
}i2cHandle_t;
static i2cHandle_t i2cHandle[I2CDEV_MAX];

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cHandle[I2CDEV_1].Handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cHandle[I2CDEV_1].Handle);
}

void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cHandle[I2CDEV_2].Handle);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cHandle[I2CDEV_2].Handle);
}

void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cHandle[I2CDEV_3].Handle);
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cHandle[I2CDEV_3].Handle);
}

#define I2C_DEFAULT_TIMEOUT 30000
static volatile uint16_t i2cErrorCount = 0;

static bool i2cOverClock;

void i2cSetOverclock(uint8_t OverClock) {
    i2cOverClock = (OverClock) ? true : false;
}

static bool i2cHandleHardwareFailure(I2CDevice bus)
{
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    i2cInit(bus);
    return false;
}

bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data, I2CDevice bus)
{
    HAL_StatusTypeDef status;
    
    if(reg_ == 0xFF)
        status = HAL_I2C_Master_Transmit(&i2cHandle[I2CDEV_MAX].Handle,addr_ << 1,data, len_, I2C_DEFAULT_TIMEOUT);
    else
        status = HAL_I2C_Mem_Write(&i2cHandle[I2CDEV_MAX].Handle,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,data, len_, I2C_DEFAULT_TIMEOUT);
        
    if(status != HAL_OK)
        return i2cHandleHardwareFailure(bus);
    
    return true;
}

bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data, I2CDevice bus)
{
    return i2cWriteBuffer(addr_, reg_, 1, &data, bus);
}

bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf, I2CDevice bus)
{
    HAL_StatusTypeDef status;
    
    if(reg_ == 0xFF)
        status = HAL_I2C_Master_Receive(&i2cHandle[I2CDEV_MAX].Handle,addr_ << 1,buf, len, I2C_DEFAULT_TIMEOUT);
    else
        status = HAL_I2C_Mem_Write(&i2cHandle[I2CDEV_MAX].Handle,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,buf, len, I2C_DEFAULT_TIMEOUT);
        
    if(status != HAL_OK)
        return i2cHandleHardwareFailure(bus);
    
    return true;
}

void i2cInit(I2CDevice bus)
{
    if (bus > I2CDEV_MAX)
        bus = I2CDEV_MAX;

    /*## Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
    RCC_PeriphCLKInitStruct.PeriphClockSelection = i2cHardwareMap[bus].clk;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = i2cHardwareMap[bus].clk_src;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    switch (bus) {
    case I2CDEV_1:
        __HAL_RCC_I2C1_CLK_ENABLE();
        break;
    case I2CDEV_2:
        __HAL_RCC_I2C2_CLK_ENABLE();
        break;
    case I2CDEV_3:
        __HAL_RCC_I2C3_CLK_ENABLE();
        break;
    default:
        break;
    }

    // clock out stuff to make sure slaves arent stuck
    // This will also configure GPIO as AF_OD at the end
    i2cUnstick(bus);
    
    GPIO_InitTypeDef  GPIO_InitStruct;
    GPIO_InitStruct.Pin       = i2cHardwareMap[bus].scl;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = i2cHardwareMap[bus].GpioAf;
    HAL_GPIO_Init(i2cHardwareMap[bus].gpioscl, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = i2cHardwareMap[bus].sda;
    HAL_GPIO_Init(i2cHardwareMap[bus].gpiosda, &GPIO_InitStruct);


    // Init I2C peripheral
    HAL_I2C_DeInit(&i2cHandle[bus].Handle);
    
    i2cHandle[bus].Handle.Instance             = i2cHardwareMap[bus].dev;
    /// TODO: HAL check if I2C timing is correct
    i2cHandle[bus].Handle.Init.Timing          = 0x00D00E28; /* (Rise time = 120ns, Fall time = 25ns) */ 
    i2cHandle[bus].Handle.Init.OwnAddress1     = 0xFF;
    i2cHandle[bus].Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    i2cHandle[bus].Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2cHandle[bus].Handle.Init.OwnAddress2     = 0xFF;
    i2cHandle[bus].Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2cHandle[bus].Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    
    
    HAL_I2C_Init(&i2cHandle[bus].Handle);
    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&i2cHandle[bus].Handle,I2C_ANALOGFILTER_ENABLE);
    
    HAL_NVIC_SetPriority(i2cHardwareMap[bus].er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    HAL_NVIC_EnableIRQ(i2cHardwareMap[bus].er_irq);
    HAL_NVIC_SetPriority(i2cHardwareMap[bus].ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
    HAL_NVIC_EnableIRQ(i2cHardwareMap[bus].ev_irq);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

static void i2cUnstick(I2CDevice bus)
{
    GPIO_TypeDef *gpioscl;
    GPIO_TypeDef *gpiosda;
    uint16_t scl, sda;
    int i;
    
    // prepare pins
    gpioscl = i2cHardwareMap[bus].gpioscl;
    scl = i2cHardwareMap[bus].scl;
    gpiosda = i2cHardwareMap[bus].gpiosda;
    sda = i2cHardwareMap[bus].sda;
    
    digitalHi(gpioscl, scl);
    digitalHi(gpiosda, sda);
    
    GPIO_InitTypeDef  GPIO_InitStruct;
    GPIO_InitStruct.Pin       = i2cHardwareMap[bus].scl;
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = i2cHardwareMap[bus].GpioAf;
    HAL_GPIO_Init(i2cHardwareMap[bus].gpioscl, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = i2cHardwareMap[bus].sda;
    HAL_GPIO_Init(i2cHardwareMap[bus].gpiosda, &GPIO_InitStruct);

    for (i = 0; i < 8; i++) {
        // Wait for any clock stretching to finish
        while (!digitalIn(gpioscl, scl))
            delayMicroseconds(10);

        // Pull low
        digitalLo(gpioscl, scl); // Set bus low
        delayMicroseconds(10);
        // Release high again
        digitalHi(gpioscl, scl); // Set bus high
        delayMicroseconds(10);
    }

    // Generate a start then stop condition
    digitalLo(gpiosda, sda); // Set bus data low
    delayMicroseconds(10);
    digitalLo(gpioscl, scl); // Set bus scl low
    delayMicroseconds(10);
    digitalHi(gpioscl, scl); // Set bus scl high
    delayMicroseconds(10);
    digitalHi(gpiosda, sda); // Set bus sda high

    // Init pins
    GPIO_InitStruct.Pin       = i2cHardwareMap[bus].scl;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = i2cHardwareMap[bus].GpioAf;
    HAL_GPIO_Init(i2cHardwareMap[bus].gpioscl, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = i2cHardwareMap[bus].sda;
    HAL_GPIO_Init(i2cHardwareMap[bus].gpiosda, &GPIO_InitStruct);
}

#endif
