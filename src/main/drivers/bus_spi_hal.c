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

#include "bus_spi.h"
#include "exti.h"
#include "io.h"
#include "io_impl.h"
#include "rcc.h"

//typedef struct spiDevice_t {
//    SPI_TypeDef *dev;
//    uint8_t GpioAf;
//    GPIO_TypeDef *gpioSck;
//    uint16_t Sck;
//    GPIO_TypeDef *gpioNss;
//    uint16_t Nss;
//    GPIO_TypeDef *gpioMiso;
//    uint16_t Miso;
//    GPIO_TypeDef *gpioMosi;
//    uint16_t Mosi;
//    uint8_t irq;
//    uint32_t speed[3];
//} spiDevice_t;

//static const spiDevice_t spiHardwareMap[] = {
//    { SPI1, MCU_SPI1_AF ,SPI1_SCK_GPIO, SPI1_SCK_PIN, SPI1_NSS_GPIO, SPI1_NSS_PIN, SPI1_MISO_GPIO, SPI1_MISO_PIN, SPI1_MOSI_GPIO, SPI1_MOSI_PIN, SPI1_IRQn, MCU_SPI1_SPEED},
//    { SPI2, MCU_SPI2_AF ,SPI2_SCK_GPIO, SPI2_SCK_PIN, SPI2_NSS_GPIO, SPI2_NSS_PIN, SPI2_MISO_GPIO, SPI2_MISO_PIN, SPI2_MOSI_GPIO, SPI2_MOSI_PIN, SPI2_IRQn, MCU_SPI2_SPEED},
//    { SPI3, MCU_SPI3_AF ,SPI3_SCK_GPIO, SPI3_SCK_PIN, SPI3_NSS_GPIO, SPI3_NSS_PIN, SPI3_MISO_GPIO, SPI3_MISO_PIN, SPI3_MOSI_GPIO, SPI3_MOSI_PIN, SPI3_IRQn, MCU_SPI3_SPEED},
//};

static spiDevice_t spiHardwareMap[] = {
    { .dev = SPI1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .rcc = RCC_APB2(SPI1), .af = MCU_SPI1_AF, false },
    { .dev = SPI2, .nss = IO_TAG(SPI2_NSS_PIN), .sck = IO_TAG(SPI2_SCK_PIN), .miso = IO_TAG(SPI2_MISO_PIN), .mosi = IO_TAG(SPI2_MOSI_PIN), .rcc = RCC_APB1(SPI2), .af = MCU_SPI2_AF, false },
    { .dev = SPI3, .nss = IO_TAG(SPI3_NSS_PIN), .sck = IO_TAG(SPI3_SCK_PIN), .miso = IO_TAG(SPI3_MISO_PIN), .mosi = IO_TAG(SPI3_MOSI_PIN), .rcc = RCC_APB1(SPI3), .af = MCU_SPI3_AF, false }
};


typedef struct{
    SPI_HandleTypeDef Handle;
}spiHandle_t;
static spiHandle_t spiHandle[SPIDEV_MAX];

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
    if (instance == SPI1)
        return SPIDEV_1;

    if (instance == SPI2)
        return SPIDEV_2;

    if (instance == SPI3)
        return SPIDEV_3;

    return SPIINVALID;
}


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


bool spiInitDevice(SPIDevice device, uint32_t divisor)
{
    spiDevice_t *spi = &(spiHardwareMap[device]);

    spiHandle[device].Handle.Instance = spi->dev;
    spiHandle[device].Handle.Init.Mode = SPI_MODE_MASTER;
    spiHandle[device].Handle.Init.Direction = SPI_DIRECTION_2LINES;
    spiHandle[device].Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    spiHandle[device].Handle.Init.NSS = SPI_NSS_SOFT;
    spiHandle[device].Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spiHandle[device].Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spiHandle[device].Handle.Init.CRCPolynomial = 7;
    spiHandle[device].Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spiHandle[device].Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spiHandle[device].Handle.Init.BaudRatePrescaler = divisor;
    spiHandle[device].Handle.Init.TIMode = SPI_TIMODE_DISABLED;

    HAL_SPI_DeInit(&spiHandle[device].Handle);
    return (HAL_SPI_Init(&spiHandle[device].Handle) == HAL_OK);
}

bool spiInit(SPIDevice device)
{
    spiDevice_t *spi = &(spiHardwareMap[device]);

#ifdef SDCARD_SPI_INSTANCE
    if (spi->dev == SDCARD_SPI_INSTANCE) {
        spi->sdcard = true;
    }
#endif
#ifdef NRF24_SPI_INSTANCE
    if (spi->dev == NRF24_SPI_INSTANCE) {
        spi->nrf24l01 = true;
    }
#endif
    
    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI, RESOURCE_SPI_SCK,  device + 1);
    IOInit(IOGetByTag(spi->miso), OWNER_SPI, RESOURCE_SPI_MISO, device + 1);
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI, RESOURCE_SPI_MOSI, device + 1);
    
    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);

    if (spi->nss) {
        IOConfigGPIOAF(IOGetByTag(spi->nss), SPI_IO_CS_CFG, spi->af);
    }
    
    switch (device)
    {
    case SPIINVALID:
        return false;
    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device, SPI_BAUDRATEPRESCALER_8);
        return true;
#else
        break;
#endif
    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device, SPI_BAUDRATEPRESCALER_8);
        return true;
#else
        break;
#endif
    case SPIDEV_3:
#ifdef USE_SPI_DEVICE_3
        spiInitDevice(device, SPI_BAUDRATEPRESCALER_8);
        return true;
#else
        break;
#endif
    }
    return false;
    
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
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data)
{
    spiTransfer(instance, &data, &data, 1);
    return data;
}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
    HAL_StatusTypeDef status;
    SPIDevice device = spiDeviceByInstance(instance);
    
    if(!out) // Tx only
    {
        status = HAL_SPI_Transmit(&spiHandle[device].Handle, (uint8_t *)in, len, SPI_DEFAULT_TIMEOUT);        
    } 
    else if(!in) // Rx only
    {
        status = HAL_SPI_Receive(&spiHandle[device].Handle, out, len, SPI_DEFAULT_TIMEOUT);
    }
    else // Tx and Rx
    {
        status = HAL_SPI_TransmitReceive(&spiHandle[device].Handle, (uint8_t *)in, out, len, SPI_DEFAULT_TIMEOUT);
    }
    
    if( status != HAL_OK)
        spiTimeoutUserCallback(device);
    
    return true;
}


void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
    SPIDevice device = spiDeviceByInstance(instance);
    uint32_t prescaler = SPI_BAUDRATEPRESCALER_8;
    
    switch (divisor) {
    case 2:
        prescaler = SPI_BAUDRATEPRESCALER_2;
        break;

    case 4:
        prescaler = SPI_BAUDRATEPRESCALER_4;
        break;

    case 8:
        prescaler = SPI_BAUDRATEPRESCALER_8;
        break;

    case 16:
        prescaler = SPI_BAUDRATEPRESCALER_16;
        break;

    case 32:
        prescaler = SPI_BAUDRATEPRESCALER_32;
        break;

    case 64:
        prescaler = SPI_BAUDRATEPRESCALER_64;
        break;

    case 128:
        prescaler = SPI_BAUDRATEPRESCALER_128;
        break;

    case 256:
        prescaler = SPI_BAUDRATEPRESCALER_256;
    }
    
    spiInitDevice(device, prescaler);
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID)
        return 0;
    return spiHardwareMap[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID)
        spiHardwareMap[device].errorCount = 0;
}

