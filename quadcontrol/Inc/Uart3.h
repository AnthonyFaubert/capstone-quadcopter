
#ifndef __UART3_H__
#define __UART3_H__

#include "stdbool.h"
#include "stdint.h"

#define UART3_RXBUF_SIZE 500
#define UART3_RXCHUNK_SIZE 5
#define UART3_TXBUF_SIZE 2000

#define PACKET_SIZE 12
#define PACKET_START_BYTE 37

// Index of where the DMA is going to put the next byte into the Uart3RxBuf FIFO
//extern int Uart3RxDmaIndex;
//extern char Uart3RxBuf[UART3_RXBUF_SIZE];

// Configures UART3 RX FIFO and DMA, call once to enable UART3 RX
void Uart3RxConfig();
// Copies len bytes from buffer to TX FIFO
void Uart3TxQueueSend(char* buffer, int len);
// Should be called as often as possible; feeds TX DMA with data from TX FIFO
// Returns false if the UART is idle even after this call, true otherwise.
int task_Uart3TxFeedDma(); // also copies everything sent to USB CDC, if available

// Should be called as often as possible; searches for a GriffinPacket in the RX FIFO and extracts it, giving it to a callback for processing
void task_Uart3RxCheckForPacket();
// Callback function to be defined by the user of the Uart3 module; is called when task_Uart3RxCheckForPacket finds a packet, invalid or not.
// Is given computed and RXed checksums and the packet buffer.
// The packet buffer will be overwritten when you return, so if you want to keep the contents, they must be copied.
extern void callback_ProcessPacket(uint8_t computedChecksum, uint8_t receivedChecksum, uint8_t* packetBuffer);

// Interrupt Service Routines for DMA
void UART3_DMA_RX_ISR();
void UART3_DMA_TX_ISR();

#endif
