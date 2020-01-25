
#include "./../IncludeFiles/TemporaryHeader.h"

void turnOnGPIO_LEDs(int clkDelay);
void turnOffGPIO_LEDs(int clkDelay);
void wait(int ticks);
void txUSART6(char sendChar);
void USART6_Config(void);

int main()
{

  // Declared Variables
  int clkDelay = 10 * 100000; // 1,000,000
//  char sendChar = '!';

  /*       GENERATED_CODE_HERE      */
  STM_Config();
  USART6_Config();

  // Function call(s) to activate necessary peripherals/interrupts, etc.

  // Main loop
  while(1)
  {
    turnOnGPIO_LEDs(clkDelay);
    turnOffGPIO_LEDs(clkDelay);
  }

  return 0;
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

void USART6_Config(void)
{
  USART6->CR1 |= USART_CR1_RXNEIE; // Enable USART6_Receive_Interrupt
}

void txUSART6(char sendChar)
{
  USART6->DR = sendChar;
}

void USART6_IT_HANDLER(void){
  txUSART6(USART6->DR);
}
