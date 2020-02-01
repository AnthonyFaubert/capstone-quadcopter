

#include "./../IncludeFiles/USART.h"

// Simple Back-Forth Communication using USART6 Peripheral
// Required input UART line.  Uses pins PC7 (USART6_RX) and PC6 (USART6_TX)
// Returns input back to sender.  Change "NumberOfBytes" to increase/decrease
// packet size

int rxReceived;
int LED_ON;

void USART6_ReturnToSender()
{
  uint32_t NumberOfBytes = 8;
  char receiveAddress[8];

  USART6->CR1 &= ~USART_CR1_RXNEIE; // Disable USART6_Receive_Interrupt
  USART6->CR1 &= ~USART_CR1_TCIE;   // Disable USART6_Transmit_Interrupt
  LED_ON = 0;

  USART6_RX_Config(NumberOfBytes, receiveAddress);

  while(1)
  {
    if(rxReceived)
    {
      txBufferUSART6(NumberOfBytes, receiveAddress);
    }
  }
}

// Much like ReturnToSender, but only allows for character transmission (
// one character at a time).  Will overflow if multiple bytes are sent at
// one time.  

void USART6_CharReturnToSender()
{
  USART6->CR1 |= USART_CR1_RXNEIE; // Enable USART6_Receive_Interrupt
  USART6->CR1 &= ~USART_CR1_TCIE;  // Disable USART6_Transmit_Interrupt
  LED_ON = 0;
  while(1){}
}

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
  txCharUSART6(USART6->DR);
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
