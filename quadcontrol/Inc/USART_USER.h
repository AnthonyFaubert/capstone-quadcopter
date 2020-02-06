

#ifndef USART_H
#define USART_H

#ifdef __cplusplus
extern "C" {
#endif

/*  Include Files  */

// Non-Local Inc
#include "./../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

// Local Inc
#include "./DMA_USER.h"

/*  Declared Functions  */

/////////////////// USART3 Asynch ///////////////////
void USART3_ReturnToSender(void);

void txBufferUSART3(uint32_t bufferSize, char * sendAddress);

void USART3_RX_Config(uint32_t bufferSize, char * targetAddress);
/////////////////////////////////////////////////////

/////////////////// USART6 Asynch ///////////////////
void USART6_ReturnToSender(void);
void USART6_CharReturnToSender(void);

void txCharUSART6(char sendChar);
void txBufferUSART6(uint32_t bufferSize, char * sendAddress);

void USART6_RX_Config(uint32_t bufferSize, char * targetAddress);

void USART6_IT_HANDLER(void);
/////////////////////////////////////////////////////

/*  Declared Variables  */
extern int rx3Received;
extern int rx6Received;

#ifdef __cplusplus
}
#endif

#endif // USART_H
