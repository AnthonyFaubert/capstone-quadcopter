
#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/*  Include Files  */

// Non-Local Inc
#include "./../../CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

// Local Inc
#include "./DMA.h"
#include "./GPIO.h"

/*  Declared Functions  */
void SPI_Init(
              uint32_t txBufferSize, uint32_t txBuffer,
              uint32_t rxBufferSize, uint32_t rxBuffer
             );

#ifdef __cplusplus
}
#endif

#endif // SPI_H
