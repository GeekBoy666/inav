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
#include <string.h>

#include "platform.h"
#include "system.h"

#include "gpio.h"

#include "sensors/sensors.h" // FIXME dependency into the main code

#include "sensor.h"
#include "accgyro.h"

#include "adc.h"
#include "adc_impl.h"

static ADC_HandleTypeDef    AdcHandle_1;
static DMA_HandleTypeDef    hdma_adc_1;

void MCU_ADC1_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcHandle_1.DMA_Handle);
}

void adcInit(drv_adc_config_t *init)
{
    uint8_t i;
    uint8_t configuredAdcChannels = 0;

    memset(&adcConfig, 0, sizeof(adcConfig));

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Alternate = 0xFF;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

#ifdef VBAT_ADC_GPIO
    if (init->enableVBat) {
        GPIO_InitStruct.Pin = VBAT_ADC_GPIO_PIN;
        HAL_GPIO_Init(VBAT_ADC_GPIO, &GPIO_InitStruct);
        
        adcConfig[ADC_BATTERY].adcChannel = VBAT_ADC_CHANNEL;
        adcConfig[ADC_BATTERY].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_BATTERY].enabled = true;
        adcConfig[ADC_BATTERY].sampleTime = ADC_SAMPLETIME_480CYCLES;
    }
#endif

#ifdef EXTERNAL1_ADC_GPIO
    if (init->enableExternal1) {
        GPIO_InitStruct.Pin = EXTERNAL1_ADC_GPIO_PIN;
        HAL_GPIO_Init(EXTERNAL1_ADC_GPIO, &GPIO_InitStruct);
        
        adcConfig[ADC_EXTERNAL1].adcChannel = EXTERNAL1_ADC_CHANNEL;
        adcConfig[ADC_EXTERNAL1].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_EXTERNAL1].enabled = true;
        adcConfig[ADC_EXTERNAL1].sampleTime = ADC_SAMPLETIME_480CYCLES;
    }
#endif

#ifdef RSSI_ADC_GPIO
    if (init->enableRSSI) {
        GPIO_InitStruct.Pin = RSSI_ADC_GPIO_PIN;
        HAL_GPIO_Init(RSSI_ADC_GPIO, &GPIO_InitStruct);
        
        adcConfig[ADC_RSSI].adcChannel = RSSI_ADC_CHANNEL;
        adcConfig[ADC_RSSI].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_RSSI].enabled = true;
        adcConfig[ADC_RSSI].sampleTime = ADC_SAMPLETIME_480CYCLES;
    }
#endif

#ifdef CURRENT_METER_ADC_GPIO
    if (init->enableCurrentMeter) {
        GPIO_InitStruct.Pin   = CURRENT_METER_ADC_GPIO_PIN;
        HAL_GPIO_Init(CURRENT_METER_ADC_GPIO, &GPIO_InitStruct);
        
        adcConfig[ADC_CURRENT].adcChannel = CURRENT_METER_ADC_CHANNEL;
        adcConfig[ADC_CURRENT].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_CURRENT].enabled = true;
        adcConfig[ADC_CURRENT].sampleTime = ADC_SAMPLETIME_480CYCLES;
    }
#endif

    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    AdcHandle_1.Instance          = ADC1;

    AdcHandle_1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV8;
    AdcHandle_1.Init.Resolution            = ADC_RESOLUTION_12B;
    AdcHandle_1.Init.ScanConvMode          = configuredAdcChannels > 1 ? ENABLE : DISABLE;
    AdcHandle_1.Init.ContinuousConvMode    = ENABLE;
    AdcHandle_1.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle_1.Init.NbrOfDiscConversion   = 0;
    AdcHandle_1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;        /* Conversion start trigged at each external event */
    AdcHandle_1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle_1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle_1.Init.NbrOfConversion       = configuredAdcChannels;
    AdcHandle_1.Init.DMAContinuousRequests = ENABLE;
    AdcHandle_1.Init.EOCSelection          = DISABLE;

    HAL_ADC_Init(&AdcHandle_1);


    hdma_adc_1.Instance = MCU_ADC1_DMA_STREAM;
  
    hdma_adc_1.Init.Channel  = MCU_ADC1_DMA_CHANNEL;
    hdma_adc_1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc_1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc_1.Init.MemInc = configuredAdcChannels > 1 ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;;
    hdma_adc_1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc_1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc_1.Init.Mode = DMA_CIRCULAR;
    hdma_adc_1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc_1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_adc_1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_adc_1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_adc_1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  
    HAL_DMA_Init(&hdma_adc_1);
  
    /* Associate the initialized DMA handle to the ADC handle */
    __HAL_LINKDMA(&AdcHandle_1, DMA_Handle, hdma_adc_1);
  
    /* NVIC configuration for DMA transfer complete interrupt */
//    HAL_NVIC_SetPriority(MCU_ADC1_DMA_STREAM_IRQn, NVIC_PRIORITY_BASE(NVIC_PRIO_ADC1_DMA), NVIC_PRIORITY_SUB(NVIC_PRIO_ADC1_DMA));
//    HAL_NVIC_EnableIRQ(MCU_ADC1_DMA_STREAM_IRQn);    

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) 
    {
        if (!adcConfig[i].enabled) {
            continue;
        }
        
        ADC_ChannelConfTypeDef sConfig;        
        sConfig.Channel      = adcConfig[i].adcChannel;
        sConfig.Rank         = rank++;
        sConfig.SamplingTime = adcConfig[i].sampleTime;
        sConfig.Offset       = 0;
        HAL_ADC_ConfigChannel(&AdcHandle_1, &sConfig);
    }

    HAL_ADC_Start_DMA(&AdcHandle_1, (uint32_t*)&adcValues, configuredAdcChannels);
}

