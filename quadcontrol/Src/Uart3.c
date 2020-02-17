

#include "Uart3.h"

int Uart3RxDmaIndex = 0;
char Uart3RxBuf[UART3_RXBUF_SIZE];

static bool TX3_InProgress = false;
static int txBufDataEndIndex = 0;
static uint8_t txBuf[UART3_TXBUF_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS; // has flag for if USB is connected
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);


void Uart3RxConfig() {
  USART3->CR1 &= ~USART_CR1_RXNEIE; // Disable USART3_Receive_Interrupt
  USART3->CR1 &= ~USART_CR1_TCIE; // Disable USART3_Transmit_Interrupt
  USART3->CR3 |= USART_CR3_DMAR; // Enable DMA Receive of USART6
  // DMA config
  DMA1_Stream1->CR &= ~DMA_SxCR_EN; // Disable DMA, Stream1
  DMA1_Stream1->CR |= DMA_SxCR_TCIE; // Activate TC Interrupt
  DMA1_Stream1->PAR = (uint32_t) &(USART3->DR); // Set Peripheral Address

  // DMA buffer config
  DMA1_Stream1->NDTR = UART3RXCHUNK_SIZE; // Reset Buffer size (Number of transfers)
  DMA1_Stream1->M0AR = (uint32_t) (&UART3RXBuf[UART3_DMA_INDEX]); // Reset address (Target Memory Address)
  
  DMA1_Stream1->CR |= DMA_SxCR_EN; // DMA ready to transfer
}

// Send Data (one-shot) via DMA to USART3
void Uart3TxStart(uint32_t bufferSize, char* sendAddress) {
  while(!(USART3->SR & USART_SR_TC));
  USART3->CR3 |=  USART_CR3_DMAT;
  USART3->SR &= ~USART_SR_TC;

  // DMA config
  TX3_InProgress = true;
  DMA1_Stream3->CR &= ~DMA_SxCR_EN;  // Turn off DMA2-Stream6
  DMA1_Stream3->CR |= DMA_SxCR_TCIE; // Activate TC Interrupt
  DMA1_Stream3->PAR = (uint32_t) &(USART3->DR); // Set Peripheral Address

  // DMA buffer config
  DMA1_Stream3->NDTR = bufferSize; // Number of transfers
  DMA1_Stream3->M0AR = (uint32_t) sendAddress; // Data-Propagation Address

  DMA1_Stream3->CR |= DMA_SxCR_EN; // Send-Data Signal
}

// Called when DMA finishes filling buffer (DMA 1, stream 1)
void UART3_DMA_RX_ISR() {
  DMA1->LIFCR |= (DMA_LIFCR_CTCIF1 + DMA_LIFCR_CHTIF1); // Clear Half-Complete and Complete Interr

  // FIFO handling
  Uart3RxDmaIndex = (Uart3RxDmaIndex + UART3_RXCHUNK_SIZE) % UART3_RXBUF_SIZE;
  DMA1_Stream1->NDTR = UART3_RXCHUNK_SIZE; // Reset Buffer size
  DMA1_Stream1->M0AR = (uint32_t) (Uart3RxBuf + Uart3RxDmaIndex); // Reset address

  DMA1_Stream1->CR |= DMA_SxCR_EN;  
}

// Clear I_Flags to prep for next transmission to USART3 (TX-DMA) (DMA 1, stream 3)
void UART3_DMA_TX_ISR() {
  DMA1->LIFCR |= (DMA_LIFCR_CTCIF3 + DMA_LIFCR_CHTIF3);
  TX3_InProgress = false;
}

// Copies len bytes from buffer to TX FIFO
void Uart3TxQueueSend(char* buffer, int len) {
  // FIXME: check for TX FIFO overflow and wait or something
  for (int i = 0; i < len; i++) {
    txBuf[txBufDataEndIndex++] = strBuf[i];
    txBufDataEndIndex %= TX_BUF_SIZE;
  }  
}

// Should be called as often as possible; feeds TX DMA with data from TX FIFO
void task_Uart3TxFeedDma() {
  static int txBufDataStartIndex = 0;
  if (txBufDataStartIndex == txBufDataEndIndex) return; // no data to send
  if (TX3_InProgress) return; // waiting for UART to finish
  
  int amountToSend;
  int startIndexAfterSend;
  if (txBufDataStartIndex > txBufDataEndIndex) {
    // FIFO wrap around
    amountToSend = TX_BUF_SIZE - txBufDataStartIndex;
    startIndexAfterSend = 0;
  } else {
    // FIFO didn't wrap around
    amountToSend = txBufDataEndIndex - txBufDataStartIndex;
    startIndexAfterSend = txBufDataEndIndex;
  }

  bool usbPresent = hUsbDeviceFS.dev_state != USBD_STATE_SUSPENDED;
  if (usbPresent && (CDC_Transmit_FS(txBuf + txBufDataStartIndex, amountToSend) != USBD_OK)) {
    // USB is connected but not ready
    return;
  } else {
    Uart3TxStart(amountToSend, (char*) (txBuf + txBufDataStartIndex));
    txBufDataStartIndex = startIndexAfterSend;
  }
}
