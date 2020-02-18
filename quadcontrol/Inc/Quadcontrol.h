
#ifndef __QUADCONTROL_H__
#define __QUADCONTROL_H__

#include "stdbool.h"
#include "stdint.h"

// Set by generated code / libraries
extern __IO uint32_t uwTick; // variable returned by HAL_GetTick()

void Quadcontrol();

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

#endif
