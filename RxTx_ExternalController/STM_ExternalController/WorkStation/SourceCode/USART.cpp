

#include "./../IncludeFiles/USART.h"

// Configure USART6 RXNEIE interrupt (receive interrupt)
void USART6_RX_Config(uint32_t bufferSize, char * targetAddress)
{
  USART6->CR1 &= ~USART_CR1_RXNEIE; // Disable USART6_Receive_Interrupt
  USART6->CR1 &= ~USART_CR1_TCIE; // Disable USART6_Transmit_Interrupt
  USART6->CR3 |= USART_CR3_DMAR; // Enable DMA Receive of USART6
  DMA_Config_USART6_RX(bufferSize, targetAddress);
}

// Send char at designated baude rate
void txCharUSART6(char sendChar)
{
  while(!(USART6->SR & USART_SR_TC));
  USART6->DR = sendChar;
}

void txBufferUSART6(uint32_t bufferSize, char * sendAddress)
{
  while(!(USART6->SR & USART_SR_TC));
  USART6->CR3 |=  USART_CR3_DMAT;
  USART6->SR &= ~USART_SR_TC;
  *sendAddress |= 0xFF;
  DMA_TX_USART6(bufferSize, sendAddress);
}

// IT_HANDLE for USART6
void USART6_IT_HANDLER(void){
  txCharUSART6(USART6->DR + 5);
  if (LED_ON)
  {
    GPIOD->BSRR |= GPIO_BSRR_BR14;
    LED_ON = 0;
  }
  else
  {
    GPIOD->BSRR |= GPIO_BSRR_BS14;
    LED_ON = 1;
  }
}
