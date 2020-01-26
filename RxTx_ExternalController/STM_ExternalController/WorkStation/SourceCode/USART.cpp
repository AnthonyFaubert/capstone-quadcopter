

#include "./../IncludeFiles/USART.h"

// Configure USART6 RXNEIE interrupt (receive interrupt)
void USART6_RX_Config(uint32_t bufferSize, char * targetAddress)
{
  USART6->CR1 |= USART_CR1_RXNEIE; // Enable USART6_Receive_Interrupt
  DMA_Config_USART6_RX(bufferSize, targetAddress);
}

// Send char at designated baude rate
void txCharUSART6(char sendChar)
{
  USART6->DR = sendChar;
}

void txBufferUSART6(uint32_t bufferSize, char * sendAddress)
{
  while(!(USART6->SR & USART_SR_TC));
  USART6->CR3 |=  USART_CR3_DMAT;
  USART6->SR &= ~USART_SR_TC;
  DMA_TX_USART6(bufferSize, sendAddress);
}

// IT_HANDLE for USART6
void USART6_IT_HANDLER(void){
  txCharUSART6(USART6->DR);
}
