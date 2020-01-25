

#include "./../IncludeFiles/DMA.h"

// Functions to consider:

// SPI ONE-SHOT Function
// ~ Make sure DMA and SPI are configured at this point ~
// 1. Load Memory Buffer
// 2. Shoot data to SPI

// Configure USART6 to send received data to a certain memory address via DMA
void DMA_Config_USART6_RX(int bufferSize, char * targetAddress)
{
  DMA2_Stream1->CR &= ~DMA_SxCR_EN; // Disable DMA, Stream1 (if on for some reason)
  DMA2_Stream1->CR &= ~DMA_SxCR_TCIE; // Disable Interrupt (temporary)
  DMA2_Stream1->NDTR = bufferSize & 0xFFFFFFFFUL; // Number of transfers
  DMA2_Stream1->M0AR = (uint32_t) targetAddress; // Target Memory Address
}

// Send Data (one-shot) via DMA to USART6
void DMA_TX_USART6(int bufferSize, char * sendAddress)
{
  DMA2_Stream6->CR &= ~DMA_SxCR_TCIE; // Disable Interrupt (temporary)
  DMA2->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6);
  //DMA2_Stream6->CR &= ~DMA_SxCR_CHSEL; // Set channel to 0
  //DMA2_Stream6->CR |= 0x6 <<  DMA_SxCR_CHSEL_Pos; // Set channel to 6
  DMA2_Stream6->NDTR = bufferSize & 0xFFFFFFFFUL; // Number of transfers
  DMA2_Stream6->PAR = (uint32_t) &(USART6->DR); // Set Peripheral Address
  DMA2_Stream6->M0AR = (uint32_t) sendAddress; // Data-Propagation Address
  DMA2_Stream6->CR |= DMA_SxCR_EN; // Send-Data Signal
}
