#ifndef PORTABLE_H
#define PORTABLE_H

/*
 * Use this file to map mcu specific functions
 */

#include "platform.h"



#ifdef STM32F745xx

// Configure UART 1
#define MCU_UART1_AF                    GPIO_AF7_USART1
#define MCU_UART1_DMA_TX_STREAM         DMA2_Stream7
#define MCU_UART1_DMA_TX_CHANNEL        DMA_CHANNEL_4
#define MCU_UART1_DMA_TX_STREAM_IRQn    DMA2_Stream7_IRQn
#define MCU_UART1_DMA_TX_IRQHandler     DMA2_Stream7_IRQHandler
#define MCU_UART1_DMA_RX_STREAM         DMA2_Stream5
#define MCU_UART1_DMA_RX_CHANNEL        DMA_CHANNEL_4
#define MCU_UART1_DMA_RX_STREAM_IRQn    DMA2_Stream5_IRQn
#define MCU_UART1_DMA_RX_IRQHandler     DMA2_Stream5_IRQHandler

/// TODO: HAL Implement definitions for other uarts
#define MCU_UART2_AF GPIO_AF7_USART2
#define MCU_UART3_AF GPIO_AF7_USART3
#define MCU_UART4_AF GPIO_AF8_UART4
#define MCU_UART5_AF GPIO_AF8_USART5
#define MCU_UART6_AF GPIO_AF8_USART6

#endif


#endif // PORTABLE_H
