

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
void DMA_Config_USART6_RX(uint32_t bufferSize, char * targetAddress);

void DMA_TX_USART6(int bufferSize, char * sendAddress);

void DMA2_STREAM1_IT_HANDLER(void);
void DMA2_STREAM6_IT_HANDLER(void);

void DMA_RX_Config_SPI1(int bufferSize, uint32_t targetAddress);

void DMA_TX_ONE_SHOT_SPI1(int bufferSize, uint32_t propagationAddress);

void DMA_TX_CONTINUOUS_SPI1(
                            int bufferSize,
                            uint32_t propagationAddress0,
                            uint32_t propagationAddress1
                           );

void DMA2_STREAM0_IT_HANDLER(void);
void DMA2_STREAM3_IT_HANDLER(void);

#ifdef __cplusplus
}
#endif

#endif // DMA_H
