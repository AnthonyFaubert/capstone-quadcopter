
#ifndef __QUADCONTROL_H__
#define __QUADCONTROL_H__

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void quadcontrol();
void emergencyStop();

#endif
