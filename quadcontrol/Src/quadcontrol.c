
//#include "main.h"
//#include "dma.h"
#include "i2c.h"
//#include "i2s.h"
//#include "spi.h"
#include "stdbool.h"
#include "stdarg.h" // allows wrapping vsprintf

#include "bno055.h"
#include "accel.h"
#include "quadcontrol.h"
#include "math.h" // for acosf() and sqrtf()

#include "VectQuatMath.h"
#include "Uart3.h"
#include "MiscPeripherals.h"
#include "PID.h"

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
  for (int i = 0; !checkButtonState(high); i++) {
    if (printPrompt && (i % 10 == 0)) {
      PRINTLN("Waiting for button press...");
    }
    txWait(20);
  }
}

#define CALIBRATE_ESCS_WAIT_MACRO(ms) txWait(ms); if (ms == 100) PRINTF(".")

// positive rotation along x-axis is pitch tape moving up
// positive rotation along y-axis is roll tape moving down
// positive rotation along z-axis is counter-clockwise looking down from above
uint8_t getQuaternion(Quaternion* quatDat) {
  const float scale = 1.0f / (1<<14);
  uint8_t readings[IMU_NUMBER_OF_BYTES];
  uint8_t status = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO<<1, BNO055_QUA_DATA_W_LSB, I2C_MEMADD_SIZE_8BIT, readings, IMU_NUMBER_OF_BYTES, 100);
  
  int16_t w = (int16_t)(readings[1] << 8) | (int16_t)(readings[0]);
  int16_t x = (int16_t)(readings[3] << 8) | (int16_t)(readings[2]);
  int16_t y = (int16_t)(readings[5] << 8) | (int16_t)(readings[4]);
  int16_t z = (int16_t)(readings[7] << 8) | (int16_t)(readings[6]);
  quatDat->w = scale * ((float) w);
  quatDat->x = scale * ((float) x);
  quatDat->y = scale * ((float) y);
  quatDat->z = scale * ((float) z);
  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {} 
  return status;
}
uint8_t getGyro(GyroData* gyroData) {
  const float scale = 1.0f / 16.0f;
  uint8_t readings[IMU_NUMBER_OF_BYTES];
  uint8_t status = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO<<1, BNO055_GYR_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, readings, IMU_NUMBER_OF_BYTES, 100);

  int16_t x = (int16_t)(readings[1] << 8) | (int16_t)(readings[0]);
  int16_t y = (int16_t)(readings[3] << 8) | (int16_t)(readings[2]);
  int16_t z = (int16_t)(readings[5] << 8) | (int16_t)(readings[4]);
  gyroData->x = scale * ((float) x);
  gyroData->y = scale * ((float) y);
  gyroData->z = scale * ((float) z);
  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {} 
  return status;
}
uint8_t getEuler(EulerData* eulerData) {
  const float scale = 1.0f / 16.0f;
  uint8_t readings[IMU_NUMBER_OF_BYTES];
  uint8_t status = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO<<1, BNO055_EUL_HEADING_LSB, I2C_MEMADD_SIZE_8BIT, readings, IMU_NUMBER_OF_BYTES, 100);
  
  int16_t x = (int16_t)(readings[1] << 8) | (int16_t)(readings[0]);
  int16_t y = (int16_t)(readings[3] << 8) | (int16_t)(readings[2]);
  int16_t z = (int16_t)(readings[5] << 8) | (int16_t)(readings[4]);
  eulerData->x = scale * ((float) x);
  eulerData->y = scale * ((float) y);
  eulerData->z = scale * ((float) z);
  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {} 
  return status;
}

void emergencyStop() {
  EmergencyShutoff();
  PRINTLN("EMERGENCY STOP ACTIVATED!");
  while (1) {
    task_Uart3TxFeedDma();
  }
}


//  SYNTAX: "b7" = bit7     b7        b6        b5        b4       B3       b2       b1       b0
// Button Format        [RESERVED, RESERVED, RESERVED, RESERVED, Button3, Button2, Button1, Button0]
uint8_t BUTTON_PRESS;
// Button3 = ?
// Button2 = ?
// Button1 = ?
// Button0 = ?
GriffinPacket InvalidPacket;
void setButtonFrame();



GriffinPacket GPacket;
int UART3_DMA_INDEX = 0;
int UART3_DMA_CHUNKS_RECVD = 0;
char UART3RXBuf[UART3RXBUF_SIZE];

int USBRXBufIndex;
char USBRXBuf[USBRXBUF_SIZE];



uint32_t InvalidCount = 0;
uint32_t ValidCount = 0;

static Quaternion joystickOrientation = {1.0f, 0.0f, 0.0f, 0.0f};
static uint32_t packetTimeout = 0, lastRXLoop = 0;
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
    Joystick2Quaternion(&joystickOrientation, GPacket);
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
    packetIndex = -1;
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


// Main program entry point
void quadcontrol() {
  BNO055_Init_I2C(&hi2c1); // FIXME: check if successful?
  PRINTLN("IMU initialized.");
  
  waitForButtonState(true, true);
  txWait(100);
  waitForButtonState(false, true);
  
  PRINTF("Calibrating ESCs.");
  CalibrateESCs();
  PRINTLN(" Done.");
  
  Uart3RxConfig();
  int usbBufIndex = 0;
  
  Quaternion imuOrientation, desiredOrientation;
  GyroData imuGyroData;
  RollPitchYaw orientationErrors;
    
  float thrust = 0.3f;
  float mVals[4];

  bool q = true; // TODO: convert to defines
  bool j = true;
  bool e = false;
  bool g = false;
  
  uint32_t scheduleButtonCheck = 0, schedulePID = 0, schedulePrintInfo = 0, worstRXstopwatch = 0, worstPIDstopwatch = 0, worstBtnStopwatch = 0, worstPoutStopwatch = 0;
  
  while (1) {
    // Sample button at 100Hz
    uint32_t btnStopwatch = uwTick;
    if (uwTick > scheduleButtonCheck) {
      scheduleButtonCheck = uwTick + 10; // 100 Hz
      if (checkButtonState(true)) emergencyStop();
    }
    btnStopwatch = uwTick - btnStopwatch;
    if (btnStopwatch > worstBtnStopwatch) worstBtnStopwatch = btnStopwatch;
    
    // Get IMU data and run PID loop, updating PWM values
    uint32_t pidStopwatch = uwTick;
    if (uwTick >= schedulePID) {
      schedulePID = uwTick + 20; // 50 Hz
      
      getQuaternion(&imuOrientation); // FIXME: check for IMU comms error!
      getGyro(&imuGyroData);
      
      QuaternionsMultiply(&desiredOrientation, joystickOrientation, TrimQuaternion);
      GetQuaternionError(&orientationErrors, imuOrientation, desiredOrientation);
      PID(mVals, orientationErrors, imuGyroData, thrust);
      if (packetTimeout != 0) {
        if (uwTick >= packetTimeout) {
          uint32_t loopDiff = uwTick - lastRXLoop;
          if (uart3bufIndex != UART3_DMA_INDEX) {
            PRINTF("FATAL: ptout. loop %d ms ago. tick=%d. HAVE data.\n", loopDiff, uwTick);
          } else {
            PRINTF("FATAL: ptout. loop %d ms ago. tick=%d. no data.\n", loopDiff, uwTick);
          }
      /* usually getting:
FATAL: ptout100ms. loop 50 ms ago. tick=33669. no data.
B=1,R=7,PI=8,PR=1,val=515,inval=0,lLoop=33619,tout=33669
      note the timeout is always 50ms(+/- 2ms) after last loop. suspicious AF. Also, this only happens when tick is almost perfectly aligned with timeout.
*/
          txWait(100);
          PRINTF("B=%d,R=%d,PI=%d,PR=%d,val=%d,inval=%d,lLoop=%d,tout=%d\n", worstBtnStopwatch, worstRXstopwatch, worstPIDstopwatch, worstPoutStopwatch, ValidCount, InvalidCount, lastRXLoop, packetTimeout);
          emergencyStop();
        }
        
        // TODO: E-stop if we're upside-down
	int mErrCode = SetMotors(mVals)
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
	  PRINTF("ERROR: mval[%d]=%.2f; thrust denied.\n", error, mVals[error]);
	}
      }
    }
    pidStopwatch = uwTick - pidStopwatch;
    if (pidStopwatch > worstPIDstopwatch) worstPIDstopwatch = pidStopwatch;
    
    
    task_Uart3TxFeedDma();
    // Check the RX FIFO for packets
    uint32_t rxStopwatch = uwTick;
    task_Uart3RxCheckForPacket();    
    rxStopwatch = uwTick - rxStopwatch;
    if (rxStopwatch > worstRXstopwatch) worstRXstopwatch = rxStopwatch;
    
    // 1 more byte for packet: got middle 10 bytes 
    
    uint32_t poutStopwatch = uwTick;
    if (uwTick >= schedulePrintInfo) {
      schedulePrintInfo = uwTick + 500; // 2 Hz
      if (packetTimeout == 0) PRINTLN("Wait 4 GPac...");
      if (q) PRINTF("QUATS: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", imuOrientation.w, imuOrientation.x, imuOrientation.y, imuOrientation.z);
      if (j) PRINTF("JOYQ: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", joystickOrientation.w, joystickOrientation.x, joystickOrientation.y, joystickOrientation.z);
      if (e) PRINTF("ERRS: R=%.2f P=%.2f Y=%.2f (all rads)\n", orientationErrors.roll, orientationErrors.pitch, orientationErrors.yaw);
      if (g) PRINTF("GYRO: X=%.2f Y=%.2f Z=%.2f\n", imuGyroData.x, imuGyroData.y, imuGyroData.z);
    }
    poutStopwatch = uwTick - poutStopwatch;
    if (poutStopwatch > worstPoutStopwatch) worstPoutStopwatch = poutStopwatch;
  }
}
