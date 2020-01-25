

#ifndef DMA_H
#define DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/*  Include Files  */

// Non-Local Inc
#include "./../../CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

// Local Inc

/*  Declared Functions  */
void DMA_Config_USART6_RX(int bufferSize, char * targetAddress);

void DMA_TX_USART6(int bufferSize, char * sendAddress);

#ifdef __cplusplus
}
#endif

#endif // DMA_H
