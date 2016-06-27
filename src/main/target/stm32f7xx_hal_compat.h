#ifndef STM32F7XX_HAL_COMPAT_H
#define STM32F7XX_HAL_COMPAT_H

#include "stm32f7xx.h"

// map some 'old' std driver functions to the new HAL

#define EXTI_GetITStatus(x) __HAL_GPIO_EXTI_GET_IT(x)
#define EXTI_ClearITPendingBit(x) __HAL_GPIO_EXTI_CLEAR_IT(x)

#define GPIO_ResetBits(port, pin) HAL_GPIO_WritePin(port,pin,false);
#define GPIO_SetBits(port, pin) HAL_GPIO_WritePin(port,pin,true);

void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);

/** @defgroup Bit_SET_and_Bit_RESET_enumeration
  * @{
  */
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;


typedef struct
{
  uint8_t NVIC_IRQChannel;                    /*!< Specifies the IRQ channel to be enabled or disabled.
                                                   This parameter can be a value of @ref IRQn_Type 
                                                   (For the complete STM32 Devices IRQ Channels list, please
                                                    refer to stm32f10x.h file) */

  uint8_t NVIC_IRQChannelPreemptionPriority;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */

  uint8_t NVIC_IRQChannelSubPriority;         /*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */

  FunctionalState NVIC_IRQChannelCmd;         /*!< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                   will be enabled or disabled. 
                                                   This parameter can be set either to ENABLE or DISABLE */   
} NVIC_InitTypeDef;


void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);


#endif // STM32F7XX_HAL_COMPAT_H
