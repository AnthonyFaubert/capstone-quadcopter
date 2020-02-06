

#include "./../IncludeFiles/SPI.h"


// Initialize SPI for data transfer
void SPI_Init()
{
  SPI1->CR2 |= SPI_CR2_TXDMAEN; // Enable DMA for TX
  SPI1->CR2 |= SPI_CR2_RXDMAEN; // Enable DMA for RX
  SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI Peripheral
  GPIOA->ODR |= GPIO_ODR_OD4; // Set NSS High
}

void SPI_TX()
{
  while(SPI1->SR & SPI_SR_BSY);
  GPIOA->ODR &= ~GPIO_ODR_OD4;
  //while(GPIOA->IDR & GPIO_IDR_ID6);
  FLUSH_DMA_SPI1();
}

void SPI_DMA_Config(
                    uint32_t rxBufferSize,
                    uint32_t rxBuffer,
                    uint32_t txBufferSize,
                    uint32_t txBuffer
                   )
{
  DMA_Config_SPI1_RX(rxBufferSize, rxBuffer);
  DMA_Config_SPI1_TX(txBufferSize, txBuffer);
}
