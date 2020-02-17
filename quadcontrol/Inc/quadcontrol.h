
#ifndef __QUADCONTROL_H__
#define __QUADCONTROL_H__

// Set by generated code / libraries
extern __IO uint32_t uwTick; // variable returned by HAL_GetTick()


void quadcontrol();

// TODO: clean up
// LOCATED IN stm32f4xx_it.c //
void setButtonFrame(void);
///////////////////////////////

typedef struct {
  float x, y, z;
} GyroData;

typedef struct {
  float x, y, z;
} EulerData;

// A representation of a quaternion rotation as a rotation of alpha along an axis defined by the x,y,z vector
typedef struct {
  float alpha, x, y, z;
} RotationAxis;

// A set of angle differences between a current heading and a desired one.
typedef struct {
  float roll, pitch, yaw;
} RollPitchYaw;

typedef struct {
  uint8_t startByte;
  int16_t leftRight;
  int16_t upDown;
  int16_t padLeftRight;
  int16_t padUpDown;
  int16_t buttons;
  uint8_t checksum;
} GriffinPacket;
#define SIZE_OF_GRIFFIN 12

extern int GPacketValid;
extern GriffinPacket GPacket;
extern void setButtonFrame();

#define UART3RXBUF_SIZE 500
#define UART3RXCHUNK_SIZE 5
extern int UART3_TX_DONE_FLAG;
extern int UART3_DMA_INDEX;
extern int UART3_DMA_CHUNKS_RECVD; // TODO: remove here and in DMA interupt
extern char UART3RXBuf[UART3RXBUF_SIZE];

#define USBRXBUF_SIZE 500
extern int USBRXBufIndex;
extern char USBRXBuf[USBRXBUF_SIZE];

#endif
