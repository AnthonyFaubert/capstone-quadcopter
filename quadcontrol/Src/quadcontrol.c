
//#include "main.h"
//#include "dma.h"
#include "i2c.h"
//#include "i2s.h"
//#include "spi.h"
#include "stdbool.h"
#include "stdarg.h" // allows wrapping vsprintf
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include "bno055.h"
#include "accel.h"
#include "quadcontrol.h"
#include "math.h" // for acosf() and sqrtf()
#include "Uart3.h"

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

bool checkButtonState(bool high) {
  if (high) {
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET;
  } else {
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
  }
}
void waitForButtonState(bool high, bool printPrompt) {
  for (int i = 0; !checkButtonState(high); i++) {
    if (printPrompt && (i % 10 == 0)) {
      PRINTLN("Waiting for button press...");
    }
    txWait(20);
  }
}
void SetPWM(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
  
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  sConfigOC.Pulse = ch1;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch2;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch3;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch4;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

// WARNING: ab != ba
// <1,0,0,0>*x = x*<1,0,0,0> = x
// This is equivalent to cross product when W=0 for both vectors
void multiplyQuaternions(Quaternion* result, Quaternion a, Quaternion b) {
  result->w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
  result->x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
  result->y = (a.w * b.y) + (a.y * b.w) + (a.z * b.x) - (a.x * b.z);
  result->z = (a.w * b.z) + (a.z * b.w) + (a.x * b.y) - (a.y * b.x);
}
// conj(a*b) = conj(b)*conj(a), normalized quaternions: q*conj(q) = q*(q^-1) = (q^-1)*q = 1
void conjugateQuaternion(Quaternion* result, Quaternion a) {
  result->w = a.w;
  result->x = -a.x;
  result->y = -a.y;
  result->z = -a.z;
}
void quaternionRotationAxis(RotationAxis* result, Quaternion q) {
  // W = cos(alpha/2), X = x*sin(alpha/2), Y = y*sin(alpha/2), Z = z*sin(alpha/2)
  // W = cos(alpha/2) => alpha/2 = acos(W), X = x*sin(alpha/2) = x*sin(acos(W)) = x*sqrt(1 - W^2) => x = X/sqrt(1 - W^2)
  result->alpha = 2.0f * acosf(q.w);
  float sqrtOneMinusWSquared = sqrtf(1.0f - q.w*q.w);
  result->x = q.x / sqrtOneMinusWSquared;
  if (result->x == NAN) {
    // then sin(alpha/2) must have been 0 => cos(alpha/2) = 1, which means X,Y,Z were 0 because of normalization,
    //  which means no rotation, so pick a default axis and then don't rotate
    result->x = 1.0f;
    result->y = 0.0f;
    result->z = 0.0f;
  } else {
    result->y = q.y / sqrtOneMinusWSquared;
    result->z = q.z / sqrtOneMinusWSquared;
  }
}
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
  multiplyQuaternions(&tmpA, correctionRotation, straight);
  straight.x = 1.0f; straight.z = 0.0f; // straight = <1,0,0>
  multiplyQuaternions(&tmpB, correctionRotation, straight);
  conjugateQuaternion(&correctionRotation, correctionRotation);
  multiplyQuaternions(&rotatedVector, tmpA, correctionRotation);
  // Overall, rotatedVector = correctionRotation * <0,0,0,1> * correctionRotation^-1 = <0,0,1> rotated by correction
  result->roll = atan2f(rotatedVector.x, rotatedVector.z);
  result->pitch = atan2f(rotatedVector.y, rotatedVector.z);
  multiplyQuaternions(&rotatedVector, tmpB, correctionRotation);
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
  SetPWM(0, 0, 0, 0);
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
const char MOTOR_CHANNEL_MAPPING[4] = {2, 3, 0, 1}; // with GND down and SIG up, left to right, the slots are: 2, 0, 1, 3
float THRUST_VECTOR_ROLL[4] = {1.0f, 0.0f, -1.0f, 0.0f};
float THRUST_VECTOR_PITCH[4] = {0.0f, -1.0f, 0.0f, 1.0f};
float THRUST_VECTOR_YAW[4] = {1.0f, -1.0f, 1.0f, -1.0f};
float GAIN_PROPORTIONAL_ROLL = 0.08f;
float GAIN_PROPORTIONAL_PITCH = 0.08f;
float GAIN_PROPORTIONAL_YAW = 0.03f;
float GAIN_DERIVATIVE_ROLL = 0.0015f;
float GAIN_DERIVATIVE_PITCH = 0.0015f;
float GAIN_DERIVATIVE_YAW = 0.0007f;
uint16_t MIN_THRUSTS[4] = {1050, 1050, 1050, 1050};
uint16_t MAX_THRUSTS[4] = {1600, 1600, 1600, 1600};

void multiply2Vectors(float* result, float* v, float* u) {
  for (int i = 0; i < 4; i++) {
    result[i] = v[i] * u[i];
  }
}
void scaleVector(float* result, float scalar, float* v) {
  for (int i = 0; i < 4; i++) {
    result[i] = scalar * v[i];
  }
}
void add3Vectors(float* result, float* v, float* u, float* w) {
  for (int i = 0; i < 4; i++) {
    result[i] = v[i] + u[i] + w[i];
  }
}
void addScalar(float*result, float scalar, float* v) {
  for (int i = 0; i < 4; i++) {
    result[i] = scalar + v[i];
  }
}
void PID(float* motorVals, RollPitchYaw rotations, GyroData gyroData, float thrust) { // TODO: derivative
  float rollVect[4], pitchVect[4], yawVect[4];
  scaleVector(rollVect, GAIN_PROPORTIONAL_ROLL * rotations.roll, THRUST_VECTOR_ROLL);
  scaleVector(pitchVect, GAIN_PROPORTIONAL_PITCH * rotations.pitch, THRUST_VECTOR_PITCH);
  scaleVector(yawVect, GAIN_PROPORTIONAL_YAW * rotations.yaw, THRUST_VECTOR_YAW);
  add3Vectors(motorVals, rollVect, pitchVect, yawVect);
  //PRINTF("MVals: %.2f, %.2f, %.2f, %.2f\n", motorVals[0], motorVals[1], motorVals[2], motorVals[3]);
  float rollRateError = 0.0f - gyroData.y; // +y = rolling in direction of positive roll torque
  float pitchRateError = 0.0f + gyroData.x; // -x = rolling in direction of positive pitch torque
  float yawRateError = 0.0f - gyroData.z; // +z = rolling in direction of positive yaw torque
  scaleVector(rollVect, GAIN_DERIVATIVE_ROLL * rollRateError, THRUST_VECTOR_ROLL);
  scaleVector(pitchVect, GAIN_DERIVATIVE_PITCH * pitchRateError, THRUST_VECTOR_PITCH);
  scaleVector(yawVect, GAIN_DERIVATIVE_YAW * yawRateError, THRUST_VECTOR_YAW);
  float derivativeMVals[4];
  add3Vectors(derivativeMVals, rollVect, pitchVect, yawVect);
  
  float average = 0.0f;
  for (int i = 0; i < 4; i++) {
    motorVals[i] += derivativeMVals[i];
    average += motorVals[i];
  }
  average /= 4.0f;
  addScalar(motorVals, thrust - average, motorVals);
}
// Returns error type: 0 = no error, 1 = thrust request not honored, -1 = non-linearity fatal error
int setMotors(float* motorVals) {
  int error = -1;
  float largest = -INFINITY;
  float smallest = INFINITY;
  for (int i = 0; i < 4; i++) {
    if (motorVals[i] > largest) largest = motorVals[i];
    if (motorVals[i] < smallest) smallest = motorVals[i];
    
    if (motorVals[i] < 0.0f || 1.0f < motorVals[i]) {
      error = i;
    }
  }
  if (error != -1) {
    if ((largest - smallest) < 1.0f) {
      if (largest > 1.0f) {
        addScalar(motorVals, 1.0f - largest, motorVals);
      } else {
        addScalar(motorVals, -smallest, motorVals);
      }
      PRINTF("ERROR: mval[%d]=%.2f; thrust denied.\n", error, motorVals[error]);
    } else {
      SetPWM(1000, 1000, 1000, 1000);
      return -1;
    }
  }

  float tmp[4];
  uint16_t thrusts[4];
  for (int i = 0; i < 4; i++) {
    tmp[i] = motorVals[i] * (float) (MAX_THRUSTS[i] - MIN_THRUSTS[i]);
    thrusts[i] = ((uint16_t) tmp[i]) + MIN_THRUSTS[i];
  }
  //PRINTF("MVsRaw: %d, %d, %d, %d\n", thrusts[MOTOR_CHANNEL_MAPPING[0]], thrusts[MOTOR_CHANNEL_MAPPING[1]], thrusts[MOTOR_CHANNEL_MAPPING[2]], thrusts[MOTOR_CHANNEL_MAPPING[3]]);
  SetPWM(thrusts[MOTOR_CHANNEL_MAPPING[0]], thrusts[MOTOR_CHANNEL_MAPPING[1]], thrusts[MOTOR_CHANNEL_MAPPING[2]], thrusts[MOTOR_CHANNEL_MAPPING[3]]);
  return (error != -1) ? 1 : 0;
}

// TODO: place somewhere else
// Set max bank angle to 45 degrees
#define PI 3.14159265358979323846f
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
  multiplyQuaternions(&tmpQuat, rollQuat, yawQuat);
  multiplyQuaternions(joyCmdQuatPtr, pitchQuat, tmpQuat);
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
      if (GPacket.buttons == JOYSTICK_BUTTON_RIGHT) multiplyQuaternions(&trimmedOrientation, QUAT_TRIM_RIGHT, trimmedOrientation);
      if (GPacket.buttons == JOYSTICK_BUTTON_LEFT) multiplyQuaternions(&trimmedOrientation, QUAT_TRIM_LEFT, trimmedOrientation);
      if (GPacket.buttons == JOYSTICK_BUTTON_UP) multiplyQuaternions(&trimmedOrientation, QUAT_TRIM_UP, trimmedOrientation);
      if (GPacket.buttons == JOYSTICK_BUTTON_DOWN) multiplyQuaternions(&trimmedOrientation, QUAT_TRIM_DOWN, trimmedOrientation);
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
  for (uint16_t pwm = 1000; pwm <= 2000; pwm += 200) {
    SetPWM(pwm, pwm, pwm, pwm);
    txWait(100);
    PRINTF(".");
  }
  SetPWM(1000, 1000, 1000, 1000);
  txWait(1000);
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
      
      multiplyQuaternions(&desiredOrientation, joystickOrientation, trimmedOrientation);
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
        if (setMotors(mVals) == -1) {
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
