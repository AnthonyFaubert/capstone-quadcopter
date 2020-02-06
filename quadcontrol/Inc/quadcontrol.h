
#ifndef __QUADCONTROL_H__
#define __QUADCONTROL_H__

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void quadcontrol();
void emergencyStop();

// LOCATED IN stm32f4xx_it.c //
void setButtonFrame(void);
///////////////////////////////

typedef struct {
  float x, y, z;
} GyroData;

typedef struct {
  float x, y, z;
} EulerData;

// W = cos(alpha/2), X = x*sin(alpha/2), Y = y*sin(alpha/2), Z = z*sin(alpha/2)
// where W,X,Y,Z are the quaternion values and x,y,z is the vector defining your direction, which alpha rotates around 
typedef struct {
  float w, x, y, z; // W,X,Y,Z, not capitalized because it's inconvenient
} Quaternion;
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
  uint8_t checksum;
  int16_t buttons;
  int16_t padUpDown;
  int16_t padLeftRight;
  int16_t upDown;
  int16_t leftRight;
} GriffinPacket;

extern int GPacketValid;
extern GriffinPacket GPacket;
extern void setButtonFrame();

#define UART3RXBUF_SIZE 2000
extern int UART3_DMA_INDEX;
extern char UART3RXBuf[UART3RXBUF_SIZE];

#endif
