
#ifndef __UART3_H__
#define __UART3_H__

#define UART3_RXBUF_SIZE 500
#define UART3_RXCHUNK_SIZE 5
#define UART3_TXBUF_SIZE 500

// Index of where the DMA is going to put the next byte into the Uart3RxBuf FIFO
extern int Uart3RxDmaIndex;
extern char Uart3RxBuf[UART3_RXBUF_SIZE];

// Configures UART3 RX FIFO and DMA, call once to enable UART3 RX
void Uart3RxConfig();
// Copies len bytes from buffer to TX FIFO
void Uart3TxQueueSend(char* buffer, int len);
// Should be called as often as possible; feeds TX DMA with data from TX FIFO
void task_Uart3TxFeedDma(); // also copies everything sent to USB CDC, if available

#endif
