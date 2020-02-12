
#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/*  Include Files  */

// Non-Local Inc
#include "stm32f407xx.h"

// Local Inc
#include "DMA.h"
#include "GPIO.h"

/*  Declared Functions  */
void SPI_Init(void);
void SPI_TX(void);
void SPI_DMA_Config(
                    uint32_t rxBufferSize,
                    uint32_t rxBuffer,
                    uint32_t txBufferSize,
                    uint32_t txBuffer
                   );

#ifdef __cplusplus
}
#endif

#endif // SPI_H
