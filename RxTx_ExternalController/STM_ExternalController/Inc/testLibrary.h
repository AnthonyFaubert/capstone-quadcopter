
// h file for testLibrary

#ifndef TEST_LIBRARY_H
#define TEST_LIBRARY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include Files */

// Non-Local Inc
#include "stm32f407xx.h"

// Local Inc
#include "USART.h"

/*  Declared Functions  */

void LED_Display(void);
void wait(int clkDelay);
void turnOnGPIO_LEDs(int clkDelay);
void turnOffGPIO_LEDs(int clkDelay);

char fourBitToHex (uint8_t hexValue);
uint16_t eightBitToHex(uint8_t charValue);
uint32_t sixteenBitToHex(uint16_t shortValue);

void newLine(void);

#ifdef __cplusplus
}
#endif


#endif // TEST_LIBRARY_H
