
//#include "main.h"
//#include "dma.h"
#include "i2c.h"
//#include "i2s.h"
//#include "spi.h"
#include "stdbool.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include "bno055.h"
#include "accel.h"
#include "quadcontrol.h"
#include "math.h" // for acosf() and sqrtf()


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
        while (HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 10) != HAL_OK) {};
 return ch;
}


uint8_t strBuf[500];

void usbprintln(const char* str) {
  uint16_t len = sprintf((char*) strBuf, "%s\n", str);
  CDC_Transmit_FS(strBuf, len);
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
      usbprintln("Waiting for button press...");
    }
    HAL_Delay(20);
  }
}
void waitWithEStopCheck(int ms) {
  for (int i = 0; i < ms/10; i++) {
    if (checkButtonState(true)) {
      emergencyStop();
    }
    HAL_Delay(10);
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
  Quaternion straight, tmpA, tmpB, rotatedVector;
  straight.w = 0.0f;
  straight.x = 0.0f;
  straight.y = 0.0f;
  straight.z = 1.0f;
  multiplyQuaternions(&tmpA, correctionRotation, straight);
  straight.x = 1.0f;
  straight.z = 0.0f;
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
  usbprintln("EMERGENCY STOP ACTIVATED!");
  while (1);
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
float GAIN_PROPORTIONAL_ROLL = 0.5f;
float GAIN_PROPORTIONAL_PITCH = 0.5f;
float GAIN_PROPORTIONAL_YAW = 0.1f;
uint16_t MIN_THRUSTS[4] = {1100, 1100, 1100, 1100}; // 1050 works
uint16_t MAX_THRUSTS[4] = {1400, 1400, 1400, 1400};

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
void PID(float* motorVals, RollPitchYaw rotations, float thrust) { // TODO: derivative
  float rollVect[4], pitchVect[4], yawVect[4];
  scaleVector(rollVect, GAIN_PROPORTIONAL_ROLL * rotations.roll, THRUST_VECTOR_ROLL);
  scaleVector(pitchVect, GAIN_PROPORTIONAL_PITCH * rotations.pitch, THRUST_VECTOR_PITCH);
  scaleVector(yawVect, GAIN_PROPORTIONAL_YAW * rotations.yaw, THRUST_VECTOR_YAW);
  add3Vectors(motorVals, rollVect, pitchVect, yawVect);
  //int len = sprintf((char*) strBuf, "MVals: %.2f, %.2f, %.2f, %.2f\r\n", motorVals[0], motorVals[1], motorVals[2], motorVals[3]);
  //CDC_Transmit_FS(strBuf, len);
  float average = 0.0f;
  for (int i = 0; i < 4; i++) {
    average += motorVals[i];
  }
  average /= 4.0f;
  addScalar(motorVals, thrust - average, motorVals);
}
void setMotors(float* motorVals) {
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
      int len = sprintf((char*) strBuf, "ERROR: motor %d's thrust (%.2f) is out of range. Thrust request denied.\r\n", error, motorVals[error]);
      CDC_Transmit_FS(strBuf, len);
      if (largest > 1.0f) {
        addScalar(motorVals, 1.0f - largest, motorVals);
      } else {
        addScalar(motorVals, -smallest, motorVals);
      }
    } else {
      int len = sprintf((char*) strBuf, "FATAL: motor %d's thrust (%.2f) is out of range! Linearity failed!\r\n", error, motorVals[error]);
      CDC_Transmit_FS(strBuf, len);
      emergencyStop();
    }
  }

  float tmp[4];
  uint16_t thrusts[4];
  for (int i = 0; i < 4; i++) {
    tmp[i] = motorVals[i] * (float) (MAX_THRUSTS[i] - MIN_THRUSTS[i]);
    thrusts[i] = ((uint16_t) tmp[i]) + MIN_THRUSTS[i];
  }
  //int len = sprintf((char*) strBuf, "MVsRaw: %d, %d, %d, %d\r\n", thrusts[MOTOR_CHANNEL_MAPPING[0]], thrusts[MOTOR_CHANNEL_MAPPING[1]], thrusts[MOTOR_CHANNEL_MAPPING[2]], thrusts[MOTOR_CHANNEL_MAPPING[3]]);
  //CDC_Transmit_FS(strBuf, len);
  SetPWM(thrusts[MOTOR_CHANNEL_MAPPING[0]], thrusts[MOTOR_CHANNEL_MAPPING[1]], thrusts[MOTOR_CHANNEL_MAPPING[2]], thrusts[MOTOR_CHANNEL_MAPPING[3]]);
}

// Main program entry point
void quadcontrol() {
  BNO055_Init_I2C(&hi2c1);
  
  usbprintln("Started.");
  waitForButtonState(true, true);
  HAL_Delay(100);
  waitForButtonState(false, true);
  
  usbprintln("Calibrating ESCs...");
  for (uint16_t pwm = 1000; pwm <= 2000; pwm += 200) {
    SetPWM(pwm, pwm, pwm, pwm);
    HAL_Delay(100);
  }
  SetPWM(1000, 1000, 1000, 1000);
  usbprintln("ESCs calibrated.");
  HAL_Delay(1000);

  Quaternion imuOrientation, desiredOrientation;
  GyroData imuGyroData;
  RollPitchYaw orientationErrors;
  desiredOrientation.w = 1.0f;
  desiredOrientation.x = 0.0f;
  desiredOrientation.y = 0.0f;
  desiredOrientation.z = 0.0f;
  int len;
  bool q = false;
  bool c = false;
  bool g = false;
  float mVals[4];
  while (1) {
    waitWithEStopCheck(20);
    
    getQuaternion(&imuOrientation);
    getGyro(&imuGyroData);
    getQuaternionError(&orientationErrors, imuOrientation, desiredOrientation);

    if (q) {
      len = sprintf((char*) strBuf, "QUATS: W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", imuOrientation.w, imuOrientation.x, imuOrientation.y, imuOrientation.z);
      CDC_Transmit_FS(strBuf, len);
    }
    if (c) {
      len = sprintf((char*) strBuf, "CMD: Roll %.2f, Pitch %.2f, Yaw %.2f (all in rads)\r\n", orientationErrors.roll, orientationErrors.pitch, orientationErrors.yaw);
      CDC_Transmit_FS(strBuf, len);
    }
    if (g) {
      len = sprintf((char*) strBuf, "GYRO: X: %.2f Y: %.2f Z: %.2f\r\n", imuGyroData.x, imuGyroData.y, imuGyroData.z);
      CDC_Transmit_FS(strBuf, len);
    }
    
    PID(mVals, orientationErrors, 0.3f);
    //for (int i = 0; i < 4; i++) mVals[i] = 0.5f;
    setMotors(mVals);
  }
}
