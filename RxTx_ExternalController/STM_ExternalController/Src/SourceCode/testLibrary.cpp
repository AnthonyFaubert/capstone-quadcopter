
// Library for Test Functions

#include "./../IncludeFiles/testLibrary.h"

////////////////////// LED Functions //////////////////////

void LED_Display()
{
  int clkDelay = 10 * 100000; // 1,000,000
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

///////////////////////////////////////////////////////////

////////////////////// Print-Functions //////////////////////

// Takes LS 4 bits of input and returns char
// representation in hex
char fourBitToHex (uint8_t hexValue)
{
  char baseChar = '0';
  hexValue &= 0x0F;
  if(hexValue >= 10)
  {
    baseChar = 'A';
    hexValue -= 0x0A;
  }
  return baseChar + hexValue;
}

// Returns hex representation of 8-bit number
uint16_t eightBitToHex(uint8_t charValue)
{
  uint16_t returnValue = 0x0000;
  returnValue |= fourBitToHex(charValue >> 4) << 8;
  returnValue |= fourBitToHex(charValue);
  return returnValue;
}

uint32_t sixteenBitToHex(uint16_t shortValue)
{
  uint32_t returnValue = 0x00000000;
  returnValue |= eightBitToHex((uint8_t) (shortValue >> 8)) << 16;
  returnValue |= eightBitToHex((uint8_t) (shortValue & 0x00FF));
  return returnValue;
}

// New Line for PUTTY interface
void newLine()
{
  char nextLine[] = { "\r\n" };
  txBufferUSART6(sizeof(nextLine), nextLine);
}

/////////////////////////////////////////////////////////////
