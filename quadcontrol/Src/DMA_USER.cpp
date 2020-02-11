

#include "./../Inc/DMA_USER.h"
#include "quadcontrol.h"

////////////////////////  DMA-UART Functions  /////////////////////////

/* Local Global Variables */
uint32_t RX3_BUFFER;
uint32_t RX3_POINTER;

uint32_t RX6_BUFFER;
uint32_t RX6_POINTER;

// Configure USART3 to send received data to a certain memory address via DMA
void DMA_Config_USART3_RX(uint32_t bufferSize, char * targetAddress)
{
  RX3_BUFFER = bufferSize & 0xFFFFFFFFUL;
  RX3_POINTER = (uint32_t) targetAddress;
  rx3Received = 0;
  DMA1_Stream1->CR &= ~DMA_SxCR_EN; // Disable DMA, Stream1
  DMA1_Stream1->CR |= DMA_SxCR_TCIE; // Activate TC Interrupt
  //DMA1_Stream1->NDTR = RX3_BUFFER; // Number of transfers
  DMA1_Stream1->PAR = (uint32_t) &(USART3->DR); // Set Peripheral Address
  //DMA1_Stream1->M0AR = RX3_POINTER; // Target Memory Address
  // FIXME PLEASE
  DMA1_Stream1->NDTR = UART3RXCHUNK_SIZE; // Reset Buffer size
  DMA1_Stream1->M0AR = (uint32_t) (&UART3RXBuf[UART3_DMA_INDEX]); // Reset address
  DMA1_Stream1->CR |= DMA_SxCR_EN; // DMA ready to transfer
}

// Send Data (one-shot) via DMA to USART6
void DMA_TX_USART3(int bufferSize, char * sendAddress)
{
  // TODO: fifo-ify
  DMA1_Stream3->CR &= ~DMA_SxCR_EN;  // Turn off DMA2-Stream6
  DMA1_Stream3->CR |= DMA_SxCR_TCIE; // Activate TC Interrupt
  DMA1_Stream3->NDTR = bufferSize & 0xFFFFFFFFUL; // Number of transfers
  DMA1_Stream3->PAR = (uint32_t) &(USART3->DR); // Set Peripheral Address
  DMA1_Stream3->M0AR = (uint32_t) sendAddress; // Data-Propagation Address
  DMA1_Stream3->CR |= DMA_SxCR_EN; // Send-Data Signal
  rx3Received = 0;
}

// Reset DMA1 to stream data from USART3 (RX-DMA)
void DMA1_STREAM1_IT_HANDLER(void)
{
  // FIXME: FOR THE LOVE OF ALL THAT IS HOLY
  DMA1->LIFCR |= (DMA_LIFCR_CTCIF1 + DMA_LIFCR_CHTIF1); // Clear Half-Complete and Complete Interr
  UART3_DMA_INDEX = (UART3_DMA_INDEX + UART3RXCHUNK_SIZE) % UART3RXBUF_SIZE;
  //DMA1_Stream1->NDTR = RX3_BUFFER; // Reset Buffer size
  DMA1_Stream1->NDTR = UART3RXCHUNK_SIZE;
  //DMA1_Stream1->M0AR = RX3_POINTER; // Reset address
  DMA1_Stream1->M0AR = (uint32_t) (UART3RXBuf + UART3_DMA_INDEX);
  rx3Received = 1;
  UART3_DMA_CHUNKS_RECVD++;
  DMA1_Stream1->CR |= DMA_SxCR_EN;
}

// Clear I_Flags to prep for next transmission to USART3 (TX-DMA)
void DMA1_STREAM3_IT_HANDLER(void) {
  UART3_TX_DONE_FLAG = 1;
  DMA1->LIFCR |= (DMA_LIFCR_CTCIF3 + DMA_LIFCR_CHTIF3);
}

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
