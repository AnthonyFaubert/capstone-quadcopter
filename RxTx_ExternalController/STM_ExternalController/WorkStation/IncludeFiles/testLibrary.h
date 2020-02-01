
// h file for testLibrary

#ifndef TEST_LIBRARY_H
#define TEST_LIBRARY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include Files */

// Non-Local Inc
#include "./../../CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

// Local Inc
// #include "./actuateMCU.h"

/*  Declared Functions  */

void LED_Display(void);
void wait(int clkDelay);
void turnOnGPIO_LEDs(int clkDelay);
void turnOffGPIO_LEDs(int clkDelay);


#ifdef __cplusplus
}
#endif


#endif // TEST_LIBRARY_H
