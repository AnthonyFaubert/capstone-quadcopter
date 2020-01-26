

#include "./../IncludeFiles/DMA.h"

// Functions to consider:

// SPI ONE-SHOT Function
// ~ Make sure DMA and SPI are configured at this point ~
// 1. Load Memory Buffer
// 2. Shoot data to SPI

/* Local Gloabl Variables */
uint32_t RX_BUFFER;
uint32_t RX_POINTER;

// Configure USART6 to send received data to a certain memory address via DMA
void DMA_Config_USART6_RX(uint32_t bufferSize, char * targetAddress)
{
  RX_BUFFER = bufferSize & 0xFFFFFFFFUL;
  RX_POINTER = (uint32_t) targetAddress;
  DMA2_Stream1->CR &= ~DMA_SxCR_EN; // Disable DMA, Stream1 (if on for some reason)
  DMA2_Stream1->NDTR = RX_BUFFER; // Number of transfers
  DMA2_Stream1->PAR = (uint32_t) & (USART6->DR); // Set Peripheral Address
  DMA2_Stream1->M0AR = RX_POINTER; // Target Memory Address
  DMA2_Stream1->CR |= DMA_SxCR_EN; // DMA ready to transfer
}

// Send Data (one-shot) via DMA to USART6
void DMA_TX_USART6(int bufferSize, char * sendAddress)
{
  DMA2_Stream6->CR &= ~DMA_SxCR_EN;
  DMA2_Stream6->CR |= DMA_SxCR_TCIE;
  DMA2_Stream6->NDTR = bufferSize & 0xFFFFFFFFUL; // Number of transfers
  DMA2_Stream6->PAR = (uint32_t) &(USART6->DR); // Set Peripheral Address
  DMA2_Stream6->M0AR = (uint32_t) sendAddress; // Data-Propagation Address
  DMA2_Stream6->CR |= DMA_SxCR_EN; // Send-Data Signal
}

void DMA2_STREAM1_IT_HANDLER(void)
{
  DMA2->LIFCR |= (DMA_LIFCR_CTCIF1 + DMA_LIFCR_CHTIF1);
  DMA2_Stream1->NDTR = RX_BUFFER;
  DMA2_Stream1->M0AR = RX_POINTER;
  DMA2_Stream1->CR |= DMA_SxCR_EN;
}

void DMA2_STREAM6_IT_HANDLER(void)
{
  DMA2->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6);
}
