
#include "stdio.h" // provides vsprintf
#include "stdarg.h" // allows wrapping vsprintf

#include "Quadcontrol.h"

#include "VectQuatMath.h"
#include "Uart3.h"
#include "MiscPeripherals.h"
#include "PID.h"
#include "IMU.h"

int USBRXBufIndex;
char USBRXBuf[USBRXBUF_SIZE];

// Maximum size of a single data chunk (no more than this many chars per printf call)
#define MAX_TX_CHUNK 100
void PRINTF(const char* fmt, ...) {
  char strBuf[MAX_TX_CHUNK];
  va_list args;
  va_start(args, fmt);
  int len = vsprintf(strBuf, fmt, args);
  Uart3TxQueueSend(strBuf, len);
  va_end(args);
}
#define PRINTLN(str) PRINTF("%s\n", str);

// Delay while allowing the TX buffer to send things
void txWait(uint32_t milliseconds) {
  uint32_t doneTime = uwTick + milliseconds;
  while (uwTick < doneTime) task_Uart3TxFeedDma();
}
void waitForButtonState(bool high, bool printPrompt) {
  for (int i = 0; !CheckButtonState(high); i++) {
    if (printPrompt && (i % 10 == 0)) {
      PRINTLN("Waiting for button press...");
    }
    txWait(20);
  }
}

#define CALIBRATE_ESCS_WAIT_MACRO(ms) txWait(ms); if (ms == 100) PRINTF(".")

// For old getQuaternion() IMU thingy:
// positive rotation along x-axis is pitch tape moving up
// positive rotation along y-axis is roll tape moving down
// positive rotation along z-axis is counter-clockwise looking down from above

void emergencyStop() {
  EmergencyShutoff();
  PRINTLN("EMERGENCY STOP ACTIVATED!");
  while (1) {
    task_Uart3TxFeedDma();
  }
}

static GriffinPacket GPacket;
static float thrust;
static Quaternion joystickOrientation = {1.0f, 0.0f, 0.0f, 0.0f};
static uint32_t packetTimeout = 0, lastRXLoop = 0;
static uint32_t ValidCount = 0, InvalidCount = 0; // TODO: rename to include "Packet"

void callback_ProcessPacket(uint8_t computedChecksum, uint8_t receivedChecksum, uint8_t* packetBuffer) {
  if (computedChecksum == receivedChecksum) { // valid packet
    ValidCount++;
    if (ValidCount%100 == 0) {
      PRINTF("Info: val/inval=%d/%d\n", ValidCount, InvalidCount);
    }
    // Decode packet contents
    GPacket.startByte = packetBuffer[0];
    GPacket.leftRight = (int16_t)(packetBuffer[1] << 8) | (int16_t)(packetBuffer[2]);
    GPacket.upDown = (int16_t)(packetBuffer[3] << 8) | (int16_t)(packetBuffer[4]);
    GPacket.padLeftRight = (int16_t)(packetBuffer[5] << 8) | (int16_t)(packetBuffer[6]);
    GPacket.padUpDown = (int16_t)(packetBuffer[7] << 8) | (int16_t)(packetBuffer[8]);
    GPacket.buttons = (int16_t)(packetBuffer[9] << 8) | (int16_t)(packetBuffer[10]);
    GPacket.checksum = packetBuffer[11];
    // Handle packet
    Joystick2Quaternion(&joystickOrientation, GPacket.leftRight, GPacket.upDown, GPacket.padLeftRight);
    thrust = (GPacket.padUpDown / 2);
    thrust = (thrust / 32768.0f) + 0.5f; // between 0.0f and 1.0f max negative/positive int16_t values
    packetTimeout = uwTick + 100;
    if (GPacket.buttons) {
      PRINTF("Btns!=0; %d\n", GPacket.buttons);
      if (GPacket.buttons == 5) emergencyStop();
      PRINTF("GPak: %d, %d, %d, %d\n", GPacket.leftRight, GPacket.upDown, GPacket.padLeftRight, GPacket.padUpDown);
      PRINTF("Thr=%.2f\n", thrust);
      JoystickApplyTrim(GPacket.buttons);
      PRINTF("TrimQ: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", TrimQuaternion.w, TrimQuaternion.x, TrimQuaternion.y, TrimQuaternion.z);
    }
  } else { // invalid packet
    InvalidCount++;
    // Print packet contents as hex and print an update on invalid/valid counters
    PRINTF("ERROR: GPac inval: ");
    for (int i = 0; i < PACKET_SIZE; i++) {
      PRINTF("%02X", packetBuffer[i]);
    }
    PRINTF(" (val/inval=%d/%d)\n", ValidCount, InvalidCount);
    
    if (InvalidCount > 20) emergencyStop();
  }
}


void task_CheckButton() {
  static uint32_t scheduleTask = 0;
    // Sample button at 100Hz
  if (uwTick > scheduleTask) {
    scheduleTask = uwTick + 10; // 100 Hz
    if (CheckButtonState(true)) emergencyStop();
  }
}

// Main program entry point
void Quadcontrol() {
  IMUInit(); // FIXME: check if successful?
  PRINTLN("IMU init.");
  Uart3RxConfig();
  PRINTLN("RX init.");
  
  waitForButtonState(true, true);
  txWait(100);
  waitForButtonState(false, true);
  
  PRINTF("Calibrating ESCs.");
  CalibrateESCs();
  PRINTLN(" Done.");
  
  Quaternion imuOrientation, desiredOrientation;
  GyroData imuGyroData;
  RollPitchYaw orientationErrors;
    
  float mVals[4];

  bool q = true; // TODO: convert to defines
  bool j = true;
  bool e = false;
  bool g = false;
  
  uint32_t schedulePID = 0, schedulePrintInfo = 0;
  
  while (1) {
    task_CheckButton();
    
    // Get IMU data and run PID loop, updating PWM values
    if (uwTick >= schedulePID) {
      schedulePID = uwTick + 20; // 50 Hz
      
      IMUGetOrientation(&imuOrientation); // FIXME: check for IMU comms error!
      IMUGetGyro(&imuGyroData);
      
      QuaternionsMultiply(&desiredOrientation, joystickOrientation, TrimQuaternion);
      GetQuaternionError(&orientationErrors, imuOrientation, desiredOrientation);
      LimitErrors(&orientationErrors);
      PID(mVals, orientationErrors, imuGyroData, thrust);
      if (packetTimeout != 0) {
        if (uwTick >= packetTimeout) {
          uint32_t loopDiff = uwTick - lastRXLoop;
	  PRINTF("FATAL: ptout. loop %d ms ago. tick=%d.", loopDiff, uwTick);
	  // TODO: remove comment:
      /* usually getting:
FATAL: ptout100ms. loop 50 ms ago. tick=33669. no data.
B=1,R=7,PI=8,PR=1,val=515,inval=0,lLoop=33619,tout=33669
      note the timeout is always 50ms(+/- 2ms) after last loop. suspicious AF. Also, this only happens when tick is almost perfectly aligned with timeout.
*/
          emergencyStop();
        }
        
        // TODO: E-stop if we're upside-down
	int mErrCode = SetMotors(mVals);
        if (mErrCode == -1) {
          PRINTLN("FATAL: nonlinearity!");
          txWait(5);
          PRINTF("mvals=[%.2f,%.2f,%.2f,%.2f]", mVals[0], mVals[1], mVals[2], mVals[3]);
          PRINTF("QUATS: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", imuOrientation.w, imuOrientation.x, imuOrientation.y, imuOrientation.z);
          txWait(2);
          PRINTF("JOYQ: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", joystickOrientation.w, joystickOrientation.x, joystickOrientation.y, joystickOrientation.z);
          PRINTF("ERRS: R=%.2f P=%.2f Y=%.2f (all rads)\n", orientationErrors.roll, orientationErrors.pitch, orientationErrors.yaw);
          txWait(2);
          PRINTF("GYRO: X=%.2f Y=%.2f Z=%.2f\n", imuGyroData.x, imuGyroData.y, imuGyroData.z);
          emergencyStop();
        } else if (mErrCode == 1) {
	  PRINTLN("ERROR: thrust denied.");
	  PRINTF("mvals=[%.2f,%.2f,%.2f,%.2f]", mVals[0], mVals[1], mVals[2], mVals[3]);
	}
      }
    }
    
    // Start TX DMA if needed
    task_Uart3TxFeedDma();
    
    // Check the RX FIFO for packets
    task_Uart3RxCheckForPacket();    
    
    if (uwTick >= schedulePrintInfo) {
      schedulePrintInfo = uwTick + 500; // 2 Hz
      if (packetTimeout == 0) PRINTLN("Wait 4 GPac...");
      if (q) PRINTF("QUATS: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", imuOrientation.w, imuOrientation.x, imuOrientation.y, imuOrientation.z);
      if (j) PRINTF("JOYQ: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", joystickOrientation.w, joystickOrientation.x, joystickOrientation.y, joystickOrientation.z);
      if (e) PRINTF("ERRS: R=%.2f P=%.2f Y=%.2f (all rads)\n", orientationErrors.roll, orientationErrors.pitch, orientationErrors.yaw);
      if (g) PRINTF("GYRO: X=%.2f Y=%.2f Z=%.2f\n", imuGyroData.x, imuGyroData.y, imuGyroData.z);
    }
  }
}
