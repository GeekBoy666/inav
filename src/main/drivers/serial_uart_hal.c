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

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "common/utils.h"
#include "gpio.h"
#include "nvic.h"
#include "inverter.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"
typedef struct{
    GPIO_TypeDef* GpioPort;
    uint32_t Pin;
    uint8_t GpioAf;
    volatile uint8_t* buff;
    uint16_t BuffSize;
    bool useDMA;
    DMA_HandleTypeDef* hdma;
    DMA_Stream_TypeDef* dmaStream;
    uint32_t dmaChannel;
    uint32_t nvicPrioDMA;
    IRQn_Type dmaIrq;
}uartDirConfig_t;

typedef struct{
    uartPort_t* uartport;
    USART_TypeDef* Instance;
    IRQn_Type InstanceIRQn;
    uartDirConfig_t tx;
    uartDirConfig_t rx;
}uartConfig_t;

static void uartIrqHandler(uartPort_t *s);
static void handleUartTxDma(uartPort_t *s);


#define UART_X_TXRX_CONFIG(X , DIR) \
    {\
        USART##X##_GPIO ,\
        USART##X##_##DIR##_PIN ,\
        MCU_UART##X##_AF ,\
        DIR####X##Buffer,\
        UART##X##_##DIR##_BUFFER_SIZE,\
        USE_USART##X##_##DIR##_DMA,\
        &hdma_##DIR####X ,\
        MCU_UART##X##_DMA_##DIR##_STREAM,\
        MCU_UART##X##_DMA_##DIR##_CHANNEL,\
        NVIC_PRIO_SERIALUART##X##_##DIR##DMA,\
        MCU_UART##X##_DMA_##DIR##_STREAM_IRQn \
    }

#define UART_X_CONFIG(X) \
    {\
        &uartPort##X,\
        USART##X,\
        USART##X##_IRQn,\
        UART_X_TXRX_CONFIG(X,TX),\
        UART_X_TXRX_CONFIG(X,RX)\
    }

#define UARTxSETUP(X) \
static uartPort_t uartPort##X;\
static volatile uint8_t RX##X##Buffer[UART##X##_RX_BUFFER_SIZE];\
static volatile uint8_t TX##X##Buffer[UART##X##_TX_BUFFER_SIZE];\
static DMA_HandleTypeDef hdma_TX##X;\
static DMA_HandleTypeDef hdma_RX##X;\
void USART##X##_IRQHandler(void)\
{\
    HAL_UART_IRQHandler(&uartPort##X.Handle);\
    uartIrqHandler(&uartPort##X);\
}\
void MCU_UART##X##_DMA_TX_IRQHandler(void)\
{\
    HAL_DMA_IRQHandler(&hdma_TX##X);\
    handleUartTxDma(&uartPort##X);\
}\
void MCU_UART##X##_DMA_RX_IRQHandler(void)\
{\
    HAL_DMA_IRQHandler(&hdma_RX##X);\
}

/// TODO: HAL Added other UARTS
#ifdef USE_USART1    
    UARTxSETUP(1)
    #define UART1CONFIG UART_X_CONFIG(1)
#endif
    
#ifdef USE_USART2    
    UARTxSETUP(2)
    #define UART2CONFIG UART_X_CONFIG(2)
#endif
    
#ifdef USE_USART3    
    UARTxSETUP(3)
    #define UART3CONFIG UART_X_CONFIG(3)
#endif
    
#ifdef USE_USART4    
    UARTxSETUP(4)
    #define UART4CONFIG UART_X_CONFIG(4)
#endif
    
#ifdef USE_USART5    
    UARTxSETUP(5)
    #define UART5CONFIG UART_X_CONFIG(5)
#endif
    
#ifdef USE_USART6    
    UARTxSETUP(6)
    #define UART6CONFIG UART_X_CONFIG(6)
#endif
    
#ifdef USE_USART7
    UARTxSETUP(7)
    #define UART7CONFIG UART_X_CONFIG(7)
#endif
    
#ifdef USE_USART8
    UARTxSETUP(8)
    #define UART8CONFIG UART_X_CONFIG(8)
#endif
    
uartConfig_t uartFindConfig(USART_TypeDef *USARTx)
{
    if (USARTx == USART1) {
        uartConfig_t cfg = UART1CONFIG;
        return cfg;
#ifdef USE_USART2
    } else if (USARTx == USART2) {
        return UART2CONFIG;
#endif
#ifdef USE_USART3
    } else if (USARTx == USART3) {
        return UART3CONFIG;
#endif
#ifdef USE_USART4
    } else if (USARTx == UART4) {
        return UART4CONFIG;
#endif
#ifdef USE_USART5
    } else if (USARTx == UART5) {
        return UART5CONFIG;
#endif
#ifdef USE_USART6
    } else if (USARTx == USART6) {
        return UART6CONFIG;
#endif
    } else {
        uartConfig_t cfg = {0};
        return cfg;
    }
}

    
static void uartOpenPortX(uartConfig_t cfg, uartPort_t* s, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    GPIO_InitTypeDef  gpio;

    s->port.vTable = uartVTable;
    
    s->port.baudRate = baudRate;
    
    s->port.rxBuffer = cfg.rx.buff;
    s->port.txBuffer = cfg.tx.buff;
    s->port.rxBufferSize = cfg.rx.BuffSize;
    s->port.txBufferSize = cfg.tx.BuffSize;
    
    
    if(cfg.rx.useDMA){
        s->rxDMAChannel = cfg.rx.dmaChannel;
        s->rxDMAStream = cfg.rx.dmaStream;
    }
    if(cfg.tx.useDMA){
        s->txDMAChannel = cfg.tx.dmaChannel;
        s->txDMAStream = cfg.tx.dmaStream;
    }

    s->Handle.Instance = cfg.Instance;

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Alternate = cfg.tx.GpioAf;
    gpio.Pull = (options & SERIAL_INVERTED) ? GPIO_PULLDOWN : GPIO_PULLUP;

    if (options & SERIAL_BIDIR) {
        gpio.Pin = cfg.tx.Pin;
        HAL_GPIO_Init(cfg.tx.GpioPort, &gpio);
        gpio.Pin = cfg.rx.Pin;
        HAL_GPIO_Init(cfg.rx.GpioPort, &gpio);
        s->Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
        s->Handle.AdvancedInit.TxPinLevelInvert = (options & SERIAL_INVERTED);
    } else {
        if (mode & MODE_TX) {
            gpio.Pin = cfg.tx.Pin;
            HAL_GPIO_Init(cfg.tx.GpioPort, &gpio);
        }

        if (mode & MODE_RX) {
            gpio.Pin = cfg.rx.Pin;
            HAL_GPIO_Init(cfg.rx.GpioPort, &gpio);
        }
    }
    
    // DMA TX Interrupt
    HAL_NVIC_SetPriority(cfg.tx.dmaIrq, NVIC_PRIORITY_BASE(cfg.tx.nvicPrioDMA), NVIC_PRIORITY_SUB(cfg.tx.nvicPrioDMA));
    HAL_NVIC_EnableIRQ(cfg.tx.dmaIrq);

    if(!cfg.rx.useDMA)
    {
        HAL_NVIC_SetPriority(cfg.InstanceIRQn, NVIC_PRIORITY_BASE(cfg.rx.nvicPrioDMA), NVIC_PRIORITY_SUB(cfg.rx.nvicPrioDMA));
        HAL_NVIC_EnableIRQ(cfg.InstanceIRQn);
    }

    return;
}

/// TODO: HAL Implement inverter in a portable way
static void usartConfigurePinInversion(uartPort_t *uartPort) {
#if !defined(INVERTER) && !defined(STM32F303xC)
    UNUSED(uartPort);
#else
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

#ifdef INVERTER
    if (inverted && uartPort->USARTx == INVERTER_USART) {
        // Enable hardware inverter if available.
        INVERTER_ON;
    }
#endif

#ifdef STM32F303xC
    uint32_t inversionPins = 0;

    if (uartPort->port.mode & MODE_TX) {
        inversionPins |= USART_InvPin_Tx;
    }
    if (uartPort->port.mode & MODE_RX) {
        inversionPins |= USART_InvPin_Rx;
    }

    USART_InvPinCmd(uartPort->USARTx, inversionPins, inverted ? ENABLE : DISABLE);
#endif
#endif
}

static void uartReconfigure(uartPort_t *uartPort)
{
    HAL_UART_DeInit(&uartPort->Handle);
    uartPort->Handle.Init.BaudRate = uartPort->port.baudRate;
    uartPort->Handle.Init.WordLength = UART_WORDLENGTH_8B;
    uartPort->Handle.Init.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOPBITS_2 : USART_STOPBITS_1;
    uartPort->Handle.Init.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE;
    uartPort->Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartPort->Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uartPort->Handle.Init.Mode = 0;

    if (uartPort->port.mode & MODE_RX)
        uartPort->Handle.Init.Mode |= UART_MODE_RX;
    if (uartPort->port.mode & MODE_TX)
        uartPort->Handle.Init.Mode |= UART_MODE_TX;

    usartConfigurePinInversion(uartPort);

    if(uartPort->port.options & SERIAL_BIDIR)
        HAL_HalfDuplex_Init(&uartPort->Handle);
    else
        HAL_UART_Init(&uartPort->Handle);
}



serialPort_t *uartOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s = NULL;
    uartConfig_t cfg = uartFindConfig(USARTx);
    
    s = cfg.uartport;
    if(s == NULL) return (serialPort_t *)s;    
    uartOpenPortX(cfg, s, baudRate, mode, options);    

    s->txDMAEmpty = true;
    
    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    s->port.callback = callback;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    uartReconfigure(s);

    // Receive DMA or IRQ
    DMA_InitTypeDef DMA_InitStructure;
    if (mode & MODE_RX) 
    {
        if (s->rxDMAStream) 
        {
            cfg.rx.hdma->Instance = cfg.rx.dmaStream;
            cfg.rx.hdma->Init.Channel = cfg.rx.dmaChannel;
            cfg.rx.hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
            cfg.rx.hdma->Init.PeriphInc = DMA_PINC_DISABLE;
            cfg.rx.hdma->Init.MemInc = DMA_MINC_ENABLE;
            cfg.rx.hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            cfg.rx.hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            cfg.rx.hdma->Init.Mode = DMA_CIRCULAR;
            cfg.rx.hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            cfg.rx.hdma->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            cfg.rx.hdma->Init.PeriphBurst = DMA_PBURST_SINGLE;
            cfg.rx.hdma->Init.MemBurst = DMA_MBURST_SINGLE;
            cfg.rx.hdma->Init.Priority = DMA_PRIORITY_MEDIUM;
            
            
            HAL_DMA_DeInit(cfg.rx.hdma);
            HAL_DMA_Init(cfg.rx.hdma);
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&cfg.uartport->Handle, hdmarx, *cfg.rx.hdma);
            
            HAL_UART_Receive_DMA(&cfg.uartport->Handle, (uint8_t *)cfg.rx.buff, cfg.rx.BuffSize);

            s->rxDMAPos = __HAL_DMA_GET_COUNTER(cfg.rx.hdma);

        } 
        else 
        {
            __HAL_UART_CLEAR_IT(&cfg.uartport->Handle, UART_FLAG_RXNE);
            __HAL_UART_ENABLE_IT(&cfg.uartport->Handle, UART_IT_RXNE);
            HAL_UART_Receive_IT(&cfg.uartport->Handle, (uint8_t *)cfg.rx.buff, cfg.rx.BuffSize); 
        }
    }

    // Transmit DMA or IRQ
    if (mode & MODE_TX) {

        if (s->txDMAStream) {
            cfg.tx.hdma->Instance = cfg.tx.dmaStream;
            cfg.tx.hdma->Init.Channel = cfg.tx.dmaChannel;
            cfg.tx.hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
            cfg.tx.hdma->Init.PeriphInc = DMA_PINC_DISABLE;
            cfg.tx.hdma->Init.MemInc = DMA_MINC_ENABLE;
            cfg.tx.hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            cfg.tx.hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            cfg.tx.hdma->Init.Mode = DMA_NORMAL;
            cfg.tx.hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            cfg.tx.hdma->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            cfg.tx.hdma->Init.PeriphBurst = DMA_PBURST_SINGLE;
            cfg.tx.hdma->Init.MemBurst = DMA_MBURST_SINGLE;
            cfg.tx.hdma->Init.Priority = DMA_PRIORITY_MEDIUM;
            
            
            HAL_DMA_DeInit(cfg.tx.hdma);
            HAL_DMA_Init(cfg.tx.hdma);
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&cfg.uartport->Handle, hdmatx, *cfg.tx.hdma);
            
            __HAL_DMA_ENABLE_IT(cfg.tx.hdma, DMA_IT_TC|DMA_IT_FE|DMA_IT_TE|DMA_IT_DME);
            __HAL_DMA_SET_COUNTER(cfg.tx.hdma, 0);
        } else {
            __HAL_UART_ENABLE_IT(&cfg.uartport->Handle, UART_IT_TXE);
        }
    }

    return (serialPort_t *)s;
}

void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    uartReconfigure(uartPort);
}

void uartSetMode(serialPort_t *instance, portMode_t mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

void uartStartTxDMA(uartPort_t *s)
{
    uint16_t size = 0;
    HAL_UART_DMAStop(&s->Handle);
   
    if (s->port.txBufferHead > s->port.txBufferTail) {
        size = s->port.txBufferHead - s->port.txBufferTail;
        s->port.txBufferTail = s->port.txBufferHead;
    } else {
        size = s->port.txBufferSize - s->port.txBufferTail;
        s->port.txBufferTail = 0;
    }
    s->txDMAEmpty = false;
    HAL_UART_Transmit_DMA(&s->Handle, (uint8_t *)&s->port.txBuffer[s->port.txBufferTail], size);

}

uint8_t uartTotalRxBytesWaiting(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t*)instance;

    if (s->rxDMAStream) {
        uint32_t rxDMAHead = __HAL_DMA_GET_COUNTER(s->Handle.hdmarx);

        if (rxDMAHead >= s->rxDMAPos) {
            return rxDMAHead - s->rxDMAPos;
        } else {
            return s->port.rxBufferSize + rxDMAHead - s->rxDMAPos;
        }
    }

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        return s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        return s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
}

uint8_t uartTotalTxBytesFree(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t*)instance;

    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }

    if (s->txDMAStream) {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must add
         * the remaining size of that in-progress transfer here instead:
         */
        bytesUsed += __HAL_DMA_GET_COUNTER(s->Handle.hdmatx);

        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if (bytesUsed >= s->port.txBufferSize - 1) {
            return 0;
        }
    }

    return (s->port.txBufferSize - 1) - bytesUsed;
}

bool isUartTransmitBufferEmpty(serialPort_t *instance)
{
    uartPort_t *s = (uartPort_t *)instance;
    if (s->txDMAStream)

        return s->txDMAEmpty;
    else
        return s->port.txBufferTail == s->port.txBufferHead;
}

uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *s = (uartPort_t *)instance;


    if (s->rxDMAStream) {

    	ch = s->port.rxBuffer[s->port.rxBufferSize - s->rxDMAPos];
        if (--s->rxDMAPos == 0)
            s->rxDMAPos = s->port.rxBufferSize;
    } else {
        ch = s->port.rxBuffer[s->port.rxBufferTail];
        if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferTail = 0;
        } else {
            s->port.rxBufferTail++;
        }
    }

    return ch;
}

void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;
    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }


    if (s->txDMAStream) {
        if (!(s->txDMAStream->CR & 1))
            uartStartTxDMA(s);
    } else {
        __HAL_UART_ENABLE_IT(&s->Handle, UART_IT_TXE);
    }
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};

void uartIrqHandler(uartPort_t *s)
{
    /// TODO: This implmentation is kind of a hack to reduce overhead otherwise generated by the HAL, there might be a better solution
    
    if (!s->rxDMAStream && (s->Handle.RxXferSize != s->Handle.RxXferCount)) {
        if (s->port.callback) {
            // The HAL has already stored the last received byte in the receive buffer we have tell it where to put the next
            s->port.callback(s->Handle.pRxBuffPtr[0]);
            s->Handle.pRxBuffPtr = (uint8_t *)s->port.rxBuffer;
        } else {
            // The HAL has already stored the last received byte in the receive buffer we have tell it where to put the next
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            s->Handle.pRxBuffPtr = (uint8_t *)&s->port.rxBuffer[s->port.rxBufferHead]; 
        }
        
        // We override the rx transfer counter to keep it going without disabling interrupts
        s->Handle.RxXferCount = s->Handle.RxXferSize;
    }

    if (!s->txDMAStream && (s->Handle.TxXferCount == 0)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            // very inefficient but will most likely never be used anyway as TX dma is enabled by default.
            HAL_UART_Transmit_IT(&s->Handle, (uint8_t *)&s->port.txBuffer[s->port.txBufferTail], 1);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        }
    }
}

static void handleUartTxDma(uartPort_t *s)
{
    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
}
