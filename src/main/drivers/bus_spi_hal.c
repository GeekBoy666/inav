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

#include "bus_spi.h"

typedef struct spiDevice_t {
    SPI_TypeDef *dev;
    uint8_t GpioAf;
    GPIO_TypeDef *gpioSck;
    uint16_t Sck;
    GPIO_TypeDef *gpioNss;
    uint16_t Nss;
    GPIO_TypeDef *gpioMiso;
    uint16_t Miso;
    GPIO_TypeDef *gpioMosi;
    uint16_t Mosi;
    uint8_t irq;
    uint32_t speed[3];
} spiDevice_t;

static const spiDevice_t spiHardwareMap[] = {
    { SPI1, MCU_SPI1_AF ,SPI1_SCK_GPIO, SPI1_SCK_PIN, SPI1_NSS_GPIO, SPI1_NSS_PIN, SPI1_MISO_GPIO, SPI1_MISO_PIN, SPI1_MOSI_GPIO, SPI1_MOSI_PIN, SPI1_IRQn, MCU_SPI1_SPEED},
    { SPI2, MCU_SPI2_AF ,SPI2_SCK_GPIO, SPI2_SCK_PIN, SPI2_NSS_GPIO, SPI2_NSS_PIN, SPI2_MISO_GPIO, SPI2_MISO_PIN, SPI2_MOSI_GPIO, SPI2_MOSI_PIN, SPI2_IRQn, MCU_SPI2_SPEED},
    { SPI3, MCU_SPI3_AF ,SPI3_SCK_GPIO, SPI3_SCK_PIN, SPI3_NSS_GPIO, SPI3_NSS_PIN, SPI3_MISO_GPIO, SPI3_MISO_PIN, SPI3_MOSI_GPIO, SPI3_MOSI_PIN, SPI3_IRQn, MCU_SPI3_SPEED},
};

typedef struct{
    SPI_HandleTypeDef Handle;
}spiHandle_t;
static spiHandle_t spiHandle[SPIDEV_MAX];


uint32_t spiErrorCount[SPIDEV_MAX];

void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiHandle[SPIDEV_1].Handle);
}

void SPI2_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiHandle[SPIDEV_2].Handle);
}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiHandle[SPIDEV_3].Handle);
}


bool SpiInitX(SPIDevice bus, uint32_t speed)
{
    spiHandle[bus].Handle.Instance = spiHardwareMap[bus].dev;
    spiHandle[bus].Handle.Init.Mode = SPI_MODE_MASTER;
    spiHandle[bus].Handle.Init.Direction = SPI_DIRECTION_2LINES;
    spiHandle[bus].Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    spiHandle[bus].Handle.Init.NSS = SPI_NSS_SOFT;
    spiHandle[bus].Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spiHandle[bus].Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spiHandle[bus].Handle.Init.CRCPolynomial = 7;
    spiHandle[bus].Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spiHandle[bus].Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spiHandle[bus].Handle.Init.BaudRatePrescaler = spiHardwareMap[bus].speed[speed];
    spiHandle[bus].Handle.Init.TIMode = SPI_TIMODE_DISABLED;

    HAL_SPI_DeInit(&spiHandle[bus].Handle);
    return (HAL_SPI_Init(&spiHandle[bus].Handle) == HAL_OK);
}

bool spiInit(SPIDevice bus)
{
#define __SPI_INIT__(x)\
    __HAL_RCC_SPI##x##_CLK_ENABLE(); \
    __HAL_RCC_SPI##x##_FORCE_RESET(); \
    __HAL_RCC_SPI##x##_RELEASE_RESET(); \
    
    // Enable SPI1 clock
    switch (bus) {
    case SPIDEV_1:
        __SPI_INIT__(1);
        break;
    case SPIDEV_2:
        __SPI_INIT__(2);
        break;
    case SPIDEV_3:
        __SPI_INIT__(3);
        break;
    default:
        return false;
        break;
    }
#undef __SPI_INIT__
    
    GPIO_InitTypeDef  GPIO_InitStruct;
    GPIO_InitStruct.Pin       = spiHardwareMap[bus].Sck;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = spiHardwareMap[bus].GpioAf;
    HAL_GPIO_Init(spiHardwareMap[bus].gpioSck, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = spiHardwareMap[bus].Mosi;
    HAL_GPIO_Init(spiHardwareMap[bus].gpioMosi, &GPIO_InitStruct);
    
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pin       = spiHardwareMap[bus].Miso;
    HAL_GPIO_Init(spiHardwareMap[bus].gpioMiso, &GPIO_InitStruct);
    
    /// TODO: HAL Implement hardware nss handling
#if 0
    GPIO_InitStruct.Pin       = spiHardwareMap[bus].Nss;
    HAL_GPIO_Init(spiHardwareMap[bus].gpioNss, &GPIO_InitStruct);
#endif
    return false;
}

uint32_t spiTimeoutUserCallback(SPIDevice instance)
{
    spiErrorCount[instance]++;
    return -1;
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPIDevice instance, uint8_t data)
{
    spiTransfer(instance, &data, &data, 1);
    return data;
}

bool spiTransfer(SPIDevice instance, uint8_t *out, const uint8_t *in, int len)
{
    HAL_StatusTypeDef status;
    
    if(!out) // Tx only
    {
        status = HAL_SPI_Transmit(&spiHandle[instance].Handle, (uint8_t *)in, len, SPI_DEFAULT_TIMEOUT);        
    } 
    else if(!in) // Rx only
    {
        status = HAL_SPI_Receive(&spiHandle[instance].Handle, out, len, SPI_DEFAULT_TIMEOUT);
    }
    else // Tx and Rx
    {
        status = HAL_SPI_TransmitReceive(&spiHandle[instance].Handle, (uint8_t *)in, out, len, SPI_DEFAULT_TIMEOUT);
    }
    
    if( status != HAL_OK)
        spiTimeoutUserCallback(instance);
    
    return true;
}


void spiSetDivisor(SPIDevice instance, SpiSpeed_t speed)
{
    SpiInitX(instance, speed);
}

uint16_t spiGetErrorCounter(SPIDevice instance)
{
    return spiErrorCount[instance];
}

void spiResetErrorCounter(SPIDevice instance)
{
    spiErrorCount[instance] = 0;
}

