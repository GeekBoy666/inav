#include "stm32f7xx_hal_compat.h"


void NVIC_Init(NVIC_InitTypeDef *NVIC_InitStruct)
{
    
    //HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
    uint32_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;
    
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(NVIC_InitStruct->NVIC_IRQChannelCmd));
    assert_param(IS_NVIC_PREEMPTION_PRIORITY(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority));  
    assert_param(IS_NVIC_SUB_PRIORITY(NVIC_InitStruct->NVIC_IRQChannelSubPriority));
    
    if (NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
    {
        HAL_NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, 
                             NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority, 
                             NVIC_InitStruct->NVIC_IRQChannelSubPriority);
        
        HAL_NVIC_EnableIRQ(NVIC_InitStruct->NVIC_IRQChannel);
    }
    else
    {
        /* Disable the Selected IRQ Channels -------------------------------------*/
        HAL_NVIC_DisableIRQ(NVIC_InitStruct->NVIC_IRQChannel);
    }
}

void GPIO_StructInit(GPIO_InitTypeDef *GPIO_InitStruct)
{
    /* Reset GPIO init structure parameters values */
    GPIO_InitStruct->Pin  = GPIO_PIN_All;
    GPIO_InitStruct->Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct->Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct->Pull = GPIO_NOPULL;
}
