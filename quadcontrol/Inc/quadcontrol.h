
#ifndef __QUADCONTROL_H__
#define __QUADCONTROL_H__

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void quadcontrol();
void emergencyStop();

typedef struct {
  float x, y, z;
} GyroData;

typedef struct {
  float x, y, z;
} EulerData;

typedef struct {
  float w, x, y, z;
} Quaternions;

#endif
