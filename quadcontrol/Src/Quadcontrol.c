
#include "stdio.h" // provides vsprintf
#include "stdarg.h" // allows wrapping vsprintf

#include "Quadcontrol.h"

#include "VectQuatMath.h"
#include "Uart3.h"
#include "MiscPeripherals.h"
#include "PID.h"
#include "IMU.h"


#define LPF_PERIOD 25
#define LPF_TYPE GyroData
#define LPF_TYPE_INIT {0.0f, 0.0f, 0.0f}
#define LPF_ADDIN(sum, b) sum.x += b.x; sum.y += b.y; sum.z += b.z;
#define LPF_SUBOUT(sum, b) sum.x -= b.x; sum.y -= b.y; sum.z -= b.z;
#define LPF_SUMDIVPERIOD(result, sum) result.x = sum.x/LPF_PERIOD; result.y = sum.y/LPF_PERIOD; result.y = sum.y/LPF_PERIOD;
LPF_TYPE lowPassFilter(LPF_TYPE newVal) {
  static LPF_TYPE LPF_FIFO[LPF_PERIOD];
  static int LPFIndex = 0;
  static LPF_TYPE LPFSum = LPF_TYPE_INIT;
  
  LPF_ADDIN(LPFSum, newVal);
  LPF_SUBOUT(LPFSum, LPF_FIFO[LPFIndex]);
  LPF_FIFO[LPFIndex] = newVal;
  LPFIndex = (LPFIndex + 1) % LPF_PERIOD;

  LPF_TYPE result;
  LPF_SUMDIVPERIOD(result, LPFSum);
  return result;
}

#define LOG_LENGTH 1000
static uint32_t logTimestamp = 0xFFFFFFFF;
static int logIndex = 0;
GyroData gyroLog[LOG_LENGTH];
Quaternion oriLog[LOG_LENGTH];
RollPitchYaw pErrorLog[LOG_LENGTH];
float mValLog[4*LOG_LENGTH];

// Maximum size of a single data chunk (no more than this many chars per printf call)
#define MAX_TX_CHUNK 100
int PRINTF(const char* fmt, ...) {
  char strBuf[MAX_TX_CHUNK];
  va_list args;
  va_start(args, fmt);
  int len = vsprintf(strBuf, fmt, args);
  Uart3TxQueueSend(strBuf, len);
  va_end(args);
  return len;
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

// Takes in whether or not to start a new log prinout. Returns true when it finishes.
bool taskPrintLog(bool start) {
  static uint32_t schedule = 0;
  static int logPrintIndex = 0;
  static int logPrintType = -1;

  if (start) {
    // Wait for the TX FIFO to flush before beginning
    logPrintIndex = 0;
    logPrintType = 0;
    schedule = uwTick + UART3_TXBUF_SIZE/12; // 115200 baud = 12.8 bytes/ms
  } else if ((logPrintType >= 0) && (uwTick >= schedule)) {
    int bytesSent = 0;

    if (logPrintIndex >= logIndex) {
      logPrintIndex = 0;
      logPrintType++;
    }
    if (logPrintIndex == 0) {
      if (logPrintType == 0) {
	bytesSent += PRINTF("LOGSTART %d\n", logTimestamp);
	bytesSent += PRINTF("#name:gyro\n#type:matrix\n#rows:%d\n#columns:3\n", logIndex);
      } else if (logPrintType == 1) {
	bytesSent += PRINTF("#name:ori\n#type:matrix\n#rows:%d\n#columns:4\n", logIndex);
      } else if (logPrintType == 2) {
	bytesSent += PRINTF("#name:pErrs\n#type:matrix\n#rows:%d\n#columns:3\n", logIndex);
      } else if (logPrintType == 3) {
	bytesSent += PRINTF("#name:mVals\n#type:matrix\n#rows:%d\n#columns:4\n", logIndex);
      } else {
	// Done, go back into waiting to start another log
	PRINTF("LOGEND\n");
	logPrintType = -1;
	return true; // finished log
      }
    }

    if (logPrintType == 0) { // gyro
      bytesSent += PRINTF("%.3f %.3f %.3f\n", gyroLog[logPrintIndex].x, gyroLog[logPrintIndex].y, gyroLog[logPrintIndex].z);
    } else if (logPrintType == 1) { // orientation
      bytesSent += PRINTF("%.4f %.4f %.4f %.4f\n", oriLog[logPrintIndex].w, oriLog[logPrintIndex].x, oriLog[logPrintIndex].y, oriLog[logPrintIndex].z);
    } else if (logPrintType == 2) { // p-errors
      bytesSent += PRINTF("%.3f %.3f %.3f\n", pErrorLog[logPrintIndex].roll, pErrorLog[logPrintIndex].pitch, pErrorLog[logPrintIndex].yaw);
    } else if (logPrintType == 3) { // mVals
      bytesSent += PRINTF("%.2f %.2f %.2f %.2f\n", mValLog[logPrintIndex*4], mValLog[logPrintIndex*4 + 1], mValLog[logPrintIndex*4 + 2], mValLog[logPrintIndex*4 + 3]);
    }
    
    logPrintIndex++;
    float sendDuration = bytesSent;
    sendDuration /= 12.8f; // 115200 baud = 12.8 bytes/ms
    schedule = uwTick + 1 + (uint32_t) sendDuration; // 1 ms extra in case of rounding errors
  }
  return false; // log not finished
}

#define CALIBRATE_ESCS_WAIT_MACRO(ms) txWait(ms); if (ms == 100) PRINTF(".")

void emergencyStop() {
  EmergencyShutoff();
  PRINTLN("EMERGENCY STOP ACTIVATED!");
  taskPrintLog(true);
  while (1) {
    taskPrintLog(false);
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
      if (GPacket.buttons == 6) {
	logTimestamp = 0xFFFFFFFF; // invalidate the timestamp
	logIndex = 0; // Reset the data log
      }
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
  lastRXLoop = uwTick;
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
  Uart3RxConfig();
  PRINTLN("RX init.");
  setIMUResetState(false);
  txWait(500);
  IMUInit(); // FIXME: check if successful?
  PRINTLN("IMU init.");
  
  waitForButtonState(true, true);
  txWait(100);
  waitForButtonState(false, true);
  
  PRINTF("Calibrating ESCs.");
  CalibrateESCs();
  PRINTLN(" Done.");
  
  Quaternion imuOrientation;
  GyroData imuGyroData;
  RollPitchYaw orientationErrors;
    
  float mVals[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  bool q = false; // TODO: convert to defines
  bool j = true;
  bool e = true;
  bool g = false;
  bool m = false;
  
  uint32_t schedulePID = 0, schedulePrintInfo = 0;

  
  while (1) {
    task_CheckButton();
    
    // Get IMU data and run PID loop, updating PWM values
    if (uwTick >= schedulePID) {
      schedulePID = uwTick + 20; // 50 Hz
      
      IMUGetOrientation(&imuOrientation); // FIXME: check for IMU comms error!
      ApplyOrientationCorrection(&imuOrientation);
      IMUGetGyro(&imuGyroData);
      ApplyGyroCorrection(&imuGyroData);
      
      GetQuaternionError(&orientationErrors, imuOrientation, joystickOrientation);
      LimitErrors(&orientationErrors);
      PID(mVals, orientationErrors, imuGyroData, thrust);
      
      if (logIndex < LOG_LENGTH) {
	// Set the initial log time
	if (logTimestamp == 0xFFFFFFFF) logTimestamp = uwTick;
        gyroLog[logIndex] = imuGyroData;
        oriLog[logIndex] = imuOrientation;
        pErrorLog[logIndex] = orientationErrors;
        for (int i = 0; i < 4; i++) {
	  mValLog[logIndex*4 + i] = mVals[i];
        }
        logIndex++;
      }

      if (packetTimeout != 0) {
        if (uwTick >= packetTimeout) {
          uint32_t time = uwTick;
	  PRINTF("FATAL: p. timeout. (diff=%d ms=%d-%d)\n", time - lastRXLoop, time, lastRXLoop);
          emergencyStop();
        }
        
        // TODO: E-stop if we're upside-down
	int mErrCode = SetMotors(mVals);
        if (mErrCode == -1) {
          PRINTLN("FATAL: nonlinearity!");
          txWait(5);
          PRINTF("mvals=[%.2f,%.2f,%.2f,%.2f]\n", mVals[0], mVals[1], mVals[2], mVals[3]);
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
      if (q) PRINTF("   QUATS: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", imuOrientation.w, imuOrientation.x, imuOrientation.y, imuOrientation.z);
      if (j) PRINTF("JOYQ: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", joystickOrientation.w, joystickOrientation.x, joystickOrientation.y, joystickOrientation.z);
      if (e) PRINTF("  ERRS: R=%.2f P=%.2f Y=%.2f (all rads)\n", orientationErrors.roll, orientationErrors.pitch, orientationErrors.yaw);
      if (g) PRINTF("GYRO: X=%.2f Y=%.2f Z=%.2f\n", imuGyroData.x, imuGyroData.y, imuGyroData.z);
      if (m) PRINTF("     mvals=[%.2f,%.2f,%.2f,%.2f]\n", mVals[0], mVals[1], mVals[2], mVals[3]);
    }
  }
}
