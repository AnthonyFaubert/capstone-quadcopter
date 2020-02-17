

#include "DMA_USER.h"

////////////////////////  DMA-UART Functions  /////////////////////////

/* Local Global Variables */
uint32_t RX6_BUFFER;
uint32_t RX6_POINTER;

// TODO: remove this file

// Configure USART6 to send received data to a certain memory address via DMA
void DMA_Config_USART6_RX(uint32_t bufferSize, char * targetAddress)
{
  RX6_BUFFER = bufferSize & 0xFFFFFFFFUL;
  RX6_POINTER = (uint32_t) targetAddress;
  rx6Received = 0;
  DMA2_Stream1->CR &= ~DMA_SxCR_EN; // Disable DMA, Stream1
  DMA2_Stream1->CR |= DMA_SxCR_TCIE; // Activate TC Interrupt
  DMA2_Stream1->NDTR = RX6_BUFFER; // Number of transfers
  DMA2_Stream1->PAR = (uint32_t) &(USART6->DR); // Set Peripheral Address
  DMA2_Stream1->M0AR = RX6_POINTER; // Target Memory Address
  DMA2_Stream1->CR |= DMA_SxCR_EN; // DMA ready to transfer
}

// Send Data (one-shot) via DMA to USART6
void DMA_TX_USART6(int bufferSize, char * sendAddress)
{
  DMA2_Stream6->CR &= ~DMA_SxCR_EN;  // Turn off DMA2-Stream6
  DMA2_Stream6->CR |= DMA_SxCR_TCIE; // Activate TC Interrupt
  DMA2_Stream6->NDTR = bufferSize & 0xFFFFFFFFUL; // Number of transfers
  DMA2_Stream6->PAR = (uint32_t) &(USART6->DR); // Set Peripheral Address
  DMA2_Stream6->M0AR = (uint32_t) sendAddress; // Data-Propagation Address
  DMA2_Stream6->CR |= DMA_SxCR_EN; // Send-Data Signal
  rx6Received = 0;
}

// Reset DMA2 to stream data from USART6 (RX-DMA)
void DMA2_STREAM1_IT_HANDLER(void)
{
  DMA2->LIFCR |= (DMA_LIFCR_CTCIF1 + DMA_LIFCR_CHTIF1); // Clear Half-Complete and Complete Interr
  DMA2_Stream1->NDTR = RX6_BUFFER; // Reset Buffer size
  DMA2_Stream1->M0AR = RX6_POINTER; // Reset address
  rx6Received = 1;
  DMA2_Stream1->CR |= DMA_SxCR_EN;
}

// Clear I_Flags to prep for next transmission to USART6 (TX-DMA)
void DMA2_STREAM6_IT_HANDLER(void)
{
  DMA2->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6);
}

/////////////////////////////////////////////////////////////////////

///////////////////////  DMA-SPI Functions  /////////////////////////

/* Local Global Veriables */
uint32_t RX_POINTER_SPI;
uint32_t RX_BUFFER_SPI;

uint32_t TX_POINTER_SPI;
uint32_t TX_BUFFER_SPI;

// Configuration for SPI-to-Mem DMA transfer
void DMA_Config_SPI1_RX(int bufferSize, uint32_t targetAddress)
{
  DMA2_Stream0->CR &= ~DMA_SxCR_TCIE; // Activate TC Interrupt
  RX_BUFFER_SPI = bufferSize & 0xFFFFFFFFUL;
  RX_POINTER_SPI = targetAddress;
  DMA2_Stream0->CR &= ~DMA_SxCR_EN; // Turn off DMA2-Stream0
  DMA2_Stream0->NDTR = RX_BUFFER_SPI; // Number of transfers
  DMA2_Stream0->PAR = (uint32_t) &(SPI1->DR); // Set Peripheral Address
  DMA2_Stream0->M0AR = RX_POINTER_SPI; // Target Memory Address
  DMA2_Stream0->CR |= DMA_SxCR_EN; // DMA ready to transfer
  DMA2_Stream0->CR |= DMA_SxCR_TCIE; // Activate TC Interrupt
}

// Configuration for Mem-to-SPI DMA transfer
void DMA_Config_SPI1_TX(int bufferSize, uint32_t propagationAddress)
{
  DMA2_Stream3->CR &= ~DMA_SxCR_EN;  // Turn off DMA2-Stream3
  TX_BUFFER_SPI = bufferSize & 0xFFFFFFFFUL;
  TX_POINTER_SPI = propagationAddress;
  DMA2_Stream3->CR |= DMA_SxCR_TCIE; // Activate TC Interrupt
  DMA2_Stream3->NDTR = TX_BUFFER_SPI; // Number of transfers
  DMA2_Stream3->PAR = (uint32_t) &(SPI1->DR); // Set Peripheral Address
  DMA2_Stream3->M0AR = TX_POINTER_SPI; // Data-Propagation Address
}

void FLUSH_DMA_SPI1()
{
  DMA2_Stream3->CR |= DMA_SxCR_EN;
}

void DMA_TX_CONTINUOUS_SPI1(
                            int bufferSize,
                            uint32_t propagationAddress0,
                            uint32_t propagationAddress1
                           )
{

}

// SPI-to-Mem transfer complete
void DMA2_STREAM0_IT_HANDLER(void)
{
  DMA2->LIFCR |= (DMA_LIFCR_CTCIF0 + DMA_LIFCR_CHTIF0); // Clear Half-Complete and Complete Interr
  //CS_Recieved = 1;
  DMA2_Stream0->NDTR = RX_BUFFER_SPI; // Reset Buffer size
  DMA2_Stream0->M0AR = RX_POINTER_SPI; // Reset address
  DMA2_Stream0->CR |= DMA_SxCR_EN;
}

// Mem-to-SPI transfer complete
void DMA2_STREAM3_IT_HANDLER(void)
{
  DMA2->LIFCR |= (DMA_LIFCR_CTCIF3 + DMA_LIFCR_CHTIF3); // Clear Half-Complete and Complete Interr
  DMA2_Stream3->NDTR = TX_BUFFER_SPI; // Reset Buffer size
  DMA2_Stream3->M0AR = TX_POINTER_SPI; // Reset address
  GPIOA->ODR |= GPIO_ODR_OD4;
}

/////////////////////////////////////////////////////////////////////
