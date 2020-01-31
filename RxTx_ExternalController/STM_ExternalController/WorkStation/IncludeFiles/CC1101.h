

#ifndef CC1101_H
#define CC1101_H

#ifdef __cplusplus
extern "C" {
#endif

/*  Include Files  */

// Non-Local Inc
#include "./../../CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

// Local Inc
#include "./USART.h"
#include "./GPIO.h"
#include "./SPI.h"

/*  Declared Functions  */
void CC1101_Configure(
                      uint32_t propagationAddress, uint32_t PA_Size
                      uint32_t targetAddress, uint32_t TA_Size
                     );

void readSPI(short readAddress);
void burstReadSPI(void);
void writeSPI(short writeAddress, short data);
void burstWriteSPI(void);

#ifdef __cplusplus
}
#endif

#endif // CC1101_H
