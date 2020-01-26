

#ifndef USART_H
#define USART_H

#ifdef __cplusplus
extern "C" {
#endif

/*  Include Files  */

// Non-Local Inc
#include "./../../CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

// Local Inc
#include "./DMA.h"

/*  Declared Functions  */

void txCharUSART6(char sendChar);
void txBufferUSART6(uint32_t bufferSize, char * sendAddress);

void USART6_RX_Config(uint32_t bufferSize, char * targetAddress);

void USART6_IT_HANDLER(void);

#ifdef __cplusplus
}
#endif

#endif // USART_H
