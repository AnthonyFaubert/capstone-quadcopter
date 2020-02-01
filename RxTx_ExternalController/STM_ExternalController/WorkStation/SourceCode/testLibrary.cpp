
// Library for Test Functions

#include "./../IncludeFiles/testLibrary.h"

void LED_Display()
{
  int clkDelay = 100 * 100000; // 1,000,000
  while(1)
  {
    turnOnGPIO_LEDs(clkDelay);
    turnOffGPIO_LEDs(clkDelay);
  }
}

void wait(int ticks)
{
  for(int i = 0; i < ticks; i++);
}

void turnOnGPIO_LEDs(int clkDelay)
{

  // Turn on Red LED (PD14)
  GPIOD->BSRR |= GPIO_BSRR_BS14; //0x1 << 5;
  wait(clkDelay);

  // Turn on Blue LED (PD15)
  GPIOD->BSRR |= GPIO_BSRR_BS15; //0x1 << 6;
  wait(clkDelay);

  // Turn on Green LED (PD12)
  GPIOD->BSRR |= GPIO_BSRR_BS12; //0x1 << 4;
  wait(clkDelay);

  // Turn on Orange LED (PD13)
  GPIOD->BSRR |= GPIO_BSRR_BS13; //0x1 << 3;
  wait(clkDelay);

}

void turnOffGPIO_LEDs(int clkDelay)
{

  // Turn off Red LED (PD14)
  GPIOD->BSRR |= GPIO_BSRR_BR14;
  wait(clkDelay);

  // Turn off Blue LED (PD15)
  GPIOD->BSRR |= GPIO_BSRR_BR15;
  wait(clkDelay);

  // Turn off Green LED (PD12)
  GPIOD->BSRR |= GPIO_BSRR_BR12;
  wait(clkDelay);

  // Turn off Orange LED (PD13)
  GPIOD->BSRR |= GPIO_BSRR_BR13;
  wait(clkDelay);
}
