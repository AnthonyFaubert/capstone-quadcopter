
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

// Assuming that the natural orientation is (FIXME), what are the roll, pitch, and yaw errors between this quaternion and the natural orientation
void getQuaternionError(RollPitchYaw* result, Quaternion actual, Quaternion desired) {
  // Rotate actual by the opposite of desired, then desired is <1,0,0,0> (rotate by q is q*a*conj(q))
  // Compute the rotation from actual to desired (invert actual to get back to straight and then go to desired)
  Quaternion correctionRotation;
  conjugateQuaternion(&actual, actual);
  multiplyQuaternions(&correctionRotation, desired, actual); // Overall rotation of (q*p) is rotate by p and then q

  // Convert the quaternion back into something we understand by using it to rotate <0,0,1> (for roll & pitch) and <1,0,0> (for yaw)
  Quaternion tmpA, tmpB, rotatedVector;
  Quaternion straight = {0.0f, 0.0f, 0.0f, 1.0f}; // straight = <0,0,1>
  QuaternionsMultiply(&tmpA, correctionRotation, straight);
  straight.x = 1.0f; straight.z = 0.0f; // straight = <1,0,0>
  QuaternionsMultiply(&tmpB, correctionRotation, straight);
  conjugateQuaternion(&correctionRotation, correctionRotation);
  QuaternionsMultiply(&rotatedVector, tmpA, correctionRotation);
  // Overall, rotatedVector = correctionRotation * <0,0,0,1> * correctionRotation^-1 = <0,0,1> rotated by correction
  result->roll = atan2f(rotatedVector.x, rotatedVector.z);
  result->pitch = atan2f(rotatedVector.y, rotatedVector.z);
  QuaternionsMultiply(&rotatedVector, tmpB, correctionRotation);
  // Overall, rotatedVector = correctionRotation * <0,1,0,0> * correctionRotation^-1 = <1,0,0> rotated by correction
  result->yaw = atan2f(rotatedVector.y, rotatedVector.x);
}

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

/*
 CW0  CCW1
CCW2   CW3
 */
// positive roll thrust will make roll tape go down, similar for rest
float THRUST_VECTOR_ROLL[4] = {1.0f, 0.0f, -1.0f, 0.0f};
float THRUST_VECTOR_PITCH[4] = {0.0f, -1.0f, 0.0f, 1.0f};
float THRUST_VECTOR_YAW[4] = {1.0f, -1.0f, 1.0f, -1.0f};
float GAIN_PROPORTIONAL_ROLL = 0.08f;
float GAIN_PROPORTIONAL_PITCH = 0.08f;
float GAIN_PROPORTIONAL_YAW = 0.03f;
float GAIN_DERIVATIVE_ROLL = 0.0015f;
float GAIN_DERIVATIVE_PITCH = 0.0015f;
float GAIN_DERIVATIVE_YAW = 0.0007f;
void PID(float* motorVals, RollPitchYaw rotations, GyroData gyroData, float thrust) { // TODO: derivative
  float rollVect[4], pitchVect[4], yawVect[4];
  VectorScale(rollVect, GAIN_PROPORTIONAL_ROLL * rotations.roll, THRUST_VECTOR_ROLL);
  VectorScale(pitchVect, GAIN_PROPORTIONAL_PITCH * rotations.pitch, THRUST_VECTOR_PITCH);
  VectorScale(yawVect, GAIN_PROPORTIONAL_YAW * rotations.yaw, THRUST_VECTOR_YAW);
  Vectors3Add(motorVals, rollVect, pitchVect, yawVect);
  //PRINTF("MVals: %.2f, %.2f, %.2f, %.2f\n", motorVals[0], motorVals[1], motorVals[2], motorVals[3]);
  float rollRateError = 0.0f - gyroData.y; // +y = rolling in direction of positive roll torque
  float pitchRateError = 0.0f + gyroData.x; // -x = rolling in direction of positive pitch torque
  float yawRateError = 0.0f - gyroData.z; // +z = rolling in direction of positive yaw torque
  VectorScale(rollVect, GAIN_DERIVATIVE_ROLL * rollRateError, THRUST_VECTOR_ROLL);
  VectorScale(pitchVect, GAIN_DERIVATIVE_PITCH * pitchRateError, THRUST_VECTOR_PITCH);
  VectorScale(yawVect, GAIN_DERIVATIVE_YAW * yawRateError, THRUST_VECTOR_YAW);
  float derivativeMVals[4];
  Vectors3Add(derivativeMVals, rollVect, pitchVect, yawVect);
  
  float average = 0.0f;
  for (int i = 0; i < 4; i++) {
    motorVals[i] += derivativeMVals[i];
    average += motorVals[i];
  }
  average /= 4.0f;
  VectorScalarAdd(motorVals, thrust - average, motorVals);
}

// TODO: place somewhere else
// Set max bank angle to 45 degrees
#define JOYSTICK_MAX_ANGLE (PI / 4.0f)
void joystick2Quaternion(Quaternion* joyCmdQuatPtr, GriffinPacket packet) {
   // NOTICE: in progress
  float roll = packet.leftRight;
  float pitch = packet.upDown;
  float yaw = packet.padLeftRight;
  yaw *= -PI / 32768.0f / 2.0f;
  roll *= JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  pitch *= -JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  Quaternion yawQuat = {cosf(yaw), 0.0f, 0.0f, sinf(yaw)};
  Quaternion rollQuat = {cosf(roll), 0.0f, sinf(roll), 0.0f};
  Quaternion pitchQuat = {cosf(pitch), sinf(pitch), 0.0f, 0.0f};
  
  // Combine rotations; yaw first, then roll, and finally pitch
  Quaternion tmpQuat;
  QuaternionsMultiply(&tmpQuat, rollQuat, yawQuat);
  QuaternionsMultiply(joyCmdQuatPtr, pitchQuat, tmpQuat);
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

#define JOYSTICK_BUTTON_RIGHT 2
#define JOYSTICK_BUTTON_LEFT 3
#define JOYSTICK_BUTTON_DOWN 1
#define JOYSTICK_BUTTON_UP 4
#define TRIM_ANGLE_PER_PRESS 1.0f * PI / 180.0f

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
    joystick2Quaternion(&joystickOrientation, GPacket);
    thrust = (GPacket.padUpDown / 2);
    thrust = (thrust / 32768.0f) + 0.5f; // between 0.0f and 1.0f max negative/positive int16_t values
    packetTimeout = uwTick + 100;
    if (GPacket.buttons) {
      PRINTF("Btns!=0; %d\n", GPacket.buttons);
      if (GPacket.buttons == 5) emergencyStop();
      PRINTF("GPak: %d, %d, %d, %d\n", GPacket.leftRight, GPacket.upDown, GPacket.padLeftRight, GPacket.padUpDown);
      PRINTF("Thr=%.2f\n", thrust);
      // NOTICE: actually apply trim quats
      if (GPacket.buttons == JOYSTICK_BUTTON_RIGHT) QuaternionsMultiply(&trimmedOrientation, QUAT_TRIM_RIGHT, trimmedOrientation);
      if (GPacket.buttons == JOYSTICK_BUTTON_LEFT) QuaternionsMultiply(&trimmedOrientation, QUAT_TRIM_LEFT, trimmedOrientation);
      if (GPacket.buttons == JOYSTICK_BUTTON_UP) QuaternionsMultiply(&trimmedOrientation, QUAT_TRIM_UP, trimmedOrientation);
      if (GPacket.buttons == JOYSTICK_BUTTON_DOWN) QuaternionsMultiply(&trimmedOrientation, QUAT_TRIM_DOWN, trimmedOrientation);
      PRINTF("TrimQ: W=%.2f X=%.2f Y=%.2f Z=%.2f\n", trimmedOrientation.w, trimmedOrientation.x, trimmedOrientation.y, trimmedOrientation.z);
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
  
  Quaternion imuOrientation, desiredOrientation, joystickOrientation;
  GyroData imuGyroData;
  RollPitchYaw orientationErrors;
  Quaternion trimmedOrientation = {1.0f, 0.0f, 0.0f, 0.0f};
  // The rotations for a single trim button press for rotating around the x and y axis, respectively
  const Quaternion QUAT_TRIM_RIGHT = {cosf(TRIM_ANGLE_PER_PRESS/2.0f), sinf(TRIM_ANGLE_PER_PRESS/2.0f), 0.0f, 0.0f};
  const Quaternion QUAT_TRIM_DOWN = {cosf(TRIM_ANGLE_PER_PRESS/2.0f), 0.0f, sinf(TRIM_ANGLE_PER_PRESS/2.0f), 0.0f};
  Quaternion QUAT_TRIM_LEFT, QUAT_TRIM_UP;
  conjugateQuaternion(&QUAT_TRIM_LEFT, QUAT_TRIM_RIGHT);
  conjugateQuaternion(&QUAT_TRIM_UP, QUAT_TRIM_DOWN);
  
    /* TODO: delete
  trimmedOrientation.w = 1.0f;
  trimmedOrientation.x = 0.0f;
  trimmedOrientation.y = 0.0f;
  trimmedOrientation.z = 0.0f;
*/
  joystickOrientation = trimmedOrientation;
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
      
      QuaternionsMultiply(&desiredOrientation, joystickOrientation, trimmedOrientation);
      getQuaternionError(&orientationErrors, imuOrientation, desiredOrientation);
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
