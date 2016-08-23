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

//#include "build_config.h"

#include "gpio.h"
#include "system.h"

#include "bus_i2c.h"
#include "nvic.h"

#ifndef SOFT_I2C

//typedef struct i2cDevice_t {
//    I2C_TypeDef *dev;
//    uint8_t GpioAf;
//    GPIO_TypeDef *gpioscl;
//    uint16_t scl;
//    GPIO_TypeDef *gpiosda;
//    uint16_t sda;
//    uint8_t ev_irq;
//    uint8_t er_irq;
//    uint32_t clk;
//    uint32_t clk_src;
//} i2cDevice_t;

//static const i2cDevice_t i2cHardwareMap[] = {
//    { I2C1, MCU_I2C1_AF ,I2C1_SCL_GPIO, I2C1_SCL_PIN, I2C1_SDA_GPIO, I2C1_SDA_PIN, I2C1_EV_IRQn, I2C1_ER_IRQn, RCC_PERIPHCLK_I2C1, RCC_I2C1CLKSOURCE_SYSCLK },
//    { I2C2, MCU_I2C2_AF ,I2C2_SCL_GPIO, I2C2_SCL_PIN, I2C2_SDA_GPIO, I2C2_SDA_PIN, I2C2_EV_IRQn, I2C2_ER_IRQn, RCC_PERIPHCLK_I2C2, RCC_I2C2CLKSOURCE_SYSCLK },
//    { I2C3, MCU_I2C3_AF ,I2C3_SCL_GPIO, I2C3_SCL_PIN, I2C3_SDA_GPIO, I2C3_SDA_PIN, I2C3_EV_IRQn, I2C3_ER_IRQn, RCC_PERIPHCLK_I2C3, RCC_I2C3CLKSOURCE_SYSCLK },
//};

#if defined(USE_I2C_PULLUP)
#define IOCFG_I2C IO_CONFIG(GPIO_Mode_AF, GPIO_SPEED_FAST, GPIO_OType_OD, GPIO_PuPd_UP)
#else
#define IOCFG_I2C IO_CONFIG(GPIO_Mode_AF, GPIO_SPEED_FAST, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#endif


static i2cDevice_t i2cHardwareMap[] = {
    { .dev = I2C1, .scl = IO_TAG(I2C1_SCL), .sda = IO_TAG(I2C1_SDA), .rcc = RCC_APB1(I2C1), .overClock = I2C1_OVERCLOCK },
    { .dev = I2C2, .scl = IO_TAG(I2C2_SCL), .sda = IO_TAG(I2C2_SDA), .rcc = RCC_APB1(I2C2), .overClock = I2C2_OVERCLOCK }
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

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    HAL_StatusTypeDef status;
    
    if(reg_ == 0xFF)
        status = HAL_I2C_Master_Transmit(&i2cHandle[I2CDEV_MAX].Handle,addr_ << 1,data, len_, I2C_DEFAULT_TIMEOUT);
    else
        status = HAL_I2C_Mem_Write(&i2cHandle[I2CDEV_MAX].Handle,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,data, len_, I2C_DEFAULT_TIMEOUT);
        
    if(status != HAL_OK)
        return i2cHandleHardwareFailure(device);
    
    return true;
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    HAL_StatusTypeDef status;
    
    if(reg_ == 0xFF)
        status = HAL_I2C_Master_Receive(&i2cHandle[device].Handle,addr_ << 1,buf, len, I2C_DEFAULT_TIMEOUT);
    else
        status = HAL_I2C_Mem_Write(&i2cHandle[device].Handle,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,buf, len, I2C_DEFAULT_TIMEOUT);
        
    if(status != HAL_OK)
        return i2cHandleHardwareFailure(device);
    
    return true;
}

void i2cInit(I2CDevice device)
{
    i2cDevice_t *i2c;
    i2c = &(i2cHardwareMap[device]);

    I2C_TypeDef *I2Cx;
    I2Cx = i2c->dev;
  
    IO_t scl = IOGetByTag(i2c->scl);
    IO_t sda = IOGetByTag(i2c->sda);

    RCC_ClockCmd(i2c->rcc, ENABLE);
    switch (device) {
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

    IOInit(scl, OWNER_I2C, RESOURCE_I2C_SCL, RESOURCE_INDEX(device));
    IOConfigGPIOAF(scl, IOCFG_I2C, MCU_I2C_AF);

    IOInit(sda, OWNER_I2C, RESOURCE_I2C_SDA, RESOURCE_INDEX(device));
    IOConfigGPIOAF(sda, IOCFG_I2C, MCU_I2C_AF);

    // Init I2C peripheral
    HAL_I2C_DeInit(&i2cHandle[device].Handle);
    
    i2cHandle[device].Handle.Instance             = i2cHardwareMap[device].dev;
    /// TODO: HAL check if I2C timing is correct
    i2cHandle[device].Handle.Init.Timing          = 0x00D00E28; /* (Rise time = 120ns, Fall time = 25ns) */ 
    i2cHandle[device].Handle.Init.OwnAddress1     = 0x00;
    i2cHandle[device].Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    i2cHandle[device].Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2cHandle[device].Handle.Init.OwnAddress2     = 0xFF;
    i2cHandle[device].Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2cHandle[device].Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    
    
    HAL_I2C_Init(&i2cHandle[device].Handle);
    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&i2cHandle[device].Handle,I2C_ANALOGFILTER_ENABLE);
    
    HAL_NVIC_SetPriority(i2cHardwareMap[device].er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    HAL_NVIC_EnableIRQ(i2cHardwareMap[device].er_irq);
    HAL_NVIC_SetPriority(i2cHardwareMap[device].ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
    HAL_NVIC_EnableIRQ(i2cHardwareMap[device].ev_irq);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

#endif
