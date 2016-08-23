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

#include "io.h"
#include "io_impl.h"
#include "rcc.h"

#include "sensors/sensors.h" // FIXME dependency into the main code

#include "sensor.h"
#include "accgyro.h"

#include "adc.h"
#include "adc_impl.h"

static ADC_HandleTypeDef    AdcHandle_1;
static DMA_HandleTypeDef    hdma_adc_1;

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
/*
    { DEFIO_TAG_E__PF3,  ADC_Channel_9  },
    { DEFIO_TAG_E__PF4,  ADC_Channel_14 },
    { DEFIO_TAG_E__PF5,  ADC_Channel_15 },
    { DEFIO_TAG_E__PF6,  ADC_Channel_4  },
    { DEFIO_TAG_E__PF7,  ADC_Channel_5  },
    { DEFIO_TAG_E__PF8,  ADC_Channel_6  },
    { DEFIO_TAG_E__PF9,  ADC_Channel_7  },
    { DEFIO_TAG_E__PF10, ADC_Channel_8  },
*/
    { DEFIO_TAG_E__PC0, ADC_CHANNEL_10 },
    { DEFIO_TAG_E__PC1, ADC_CHANNEL_11 },
    { DEFIO_TAG_E__PC2, ADC_CHANNEL_12 },
    { DEFIO_TAG_E__PC3, ADC_CHANNEL_13 },
    { DEFIO_TAG_E__PC4, ADC_CHANNEL_14 },
    { DEFIO_TAG_E__PC5, ADC_CHANNEL_15 },
    { DEFIO_TAG_E__PB0, ADC_CHANNEL_8  },
    { DEFIO_TAG_E__PB1, ADC_CHANNEL_9  },
    { DEFIO_TAG_E__PA0, ADC_CHANNEL_0  },
    { DEFIO_TAG_E__PA1, ADC_CHANNEL_1  },
    { DEFIO_TAG_E__PA2, ADC_CHANNEL_2  },
    { DEFIO_TAG_E__PA3, ADC_CHANNEL_3  },
    { DEFIO_TAG_E__PA4, ADC_CHANNEL_4  },
    { DEFIO_TAG_E__PA5, ADC_CHANNEL_5  },
    { DEFIO_TAG_E__PA6, ADC_CHANNEL_6  },
    { DEFIO_TAG_E__PA7, ADC_CHANNEL_7  },
};

void MCU_ADC1_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcHandle_1.DMA_Handle);
}

void adcInit(drv_adc_config_t *init)
{
    uint8_t i;
    uint8_t configuredAdcChannels = 0;

    memset(&adcConfig, 0, sizeof(adcConfig));

#ifdef VBAT_ADC_PIN
    if (init->enableVBat) {
        adcConfig[ADC_BATTERY].tag = IO_TAG(VBAT_ADC_PIN); //VBAT_ADC_CHANNEL;
    }
#endif

#ifdef RSSI_ADC_PIN
    if (init->enableRSSI) {
        adcConfig[ADC_RSSI].tag = IO_TAG(RSSI_ADC_PIN);  //RSSI_ADC_CHANNEL;
    }
#endif

#ifdef EXTERNAL1_ADC_PIN
    if (init->enableExternal1) {
        adcConfig[ADC_EXTERNAL1].tag = IO_TAG(EXTERNAL1_ADC_PIN); //EXTERNAL1_ADC_CHANNEL;
    }
#endif

#ifdef CURRENT_METER_ADC_PIN
    if (init->enableCurrentMeter) {
        adcConfig[ADC_CURRENT].tag = IO_TAG(CURRENT_METER_ADC_PIN);  //CURRENT_METER_ADC_CHANNEL;
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
        
        IOInit(IOGetByTag(adcConfig[i].tag), OWNER_ADC, RESOURCE_ADC_BATTERY + i, 0);
        IOConfigGPIO(IOGetByTag(adcConfig[i].tag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
        ADC_ChannelConfTypeDef sConfig;        
        sConfig.Channel      = adcChannelByTag(adcConfig[i].tag);
        sConfig.Rank         = rank++;
        sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        sConfig.Offset       = 0;
        HAL_ADC_ConfigChannel(&AdcHandle_1, &sConfig);
    }

    HAL_ADC_Start_DMA(&AdcHandle_1, (uint32_t*)&adcValues, configuredAdcChannels);
}

