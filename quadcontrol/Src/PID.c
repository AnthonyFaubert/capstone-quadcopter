
#include "PID.h"
#include "VectQuatMath.h"
#include "math.h"

Quaternion TrimQuaternion = {1.0f, 0.0f, 0.0f, 0.0f};
static int forwardTrim = 0, rightTrim = 0;

void ApplyOrientationCorrection(Quaternion* orientation) {
  static Quaternion imuOrientationCorrection = ORIENTATION_CORRECTION_QUATERNION;
  QuaternionsMultiply(orientation, *orientation, imuOrientationCorrection);
}

void ApplyGyroCorrection(GyroData* gyroData) {
  float pitch = GYRO_CORRECTION_COSINE * gyroData->x + GYRO_CORRECTION_SINE * gyroData->y;
  float roll = GYRO_CORRECTION_SINE * gyroData->x - GYRO_CORRECTION_COSINE * gyroData->y;
  gyroData->x = roll;
  gyroData->y = pitch;
}

// Converts a quaternion rotation to an Euler rotation representation
RollPitchYaw Quaternion2Euler(Quaternion q) {
  // Contents of this function are from Wikipedia with minor modifications
  RollPitchYaw ypr;

  // roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  ypr.roll = atan2f(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (sinp > 1.0f) { // use 90 degrees if out of range
    ypr.pitch = PI / 2.0f;
  } else if (sinp < -1.0f) {
    ypr.pitch = -PI / 2.0f;
  } else {
    ypr.pitch = asinf(sinp);
  }

  // yaw (z-axis rotation)
  float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  ypr.yaw = atan2f(siny_cosp, cosy_cosp);

  return ypr;
}

// Gives rotation required to get from the actual orientation to the desired orientation
Quaternion GetQuaternionError(Quaternion earth2Actual, Quaternion earth2Desired) {
  Quaternion actual2Earth = QuaternionConjugate(earth2Actual);
  Quaternion actual2Desired;
  QuaternionsMultiply(&actual2Desired, actual2Earth, earth2Desired);
  return actual2Desired;
}

// TODO: better approach, or just remove
Quaternion imuRollPitch, imuYaw = {0.0f, 0.0f, 0.0f, 0.0f}; // temporary globals for printouts
RollPitchYaw NewControl(Quaternion imuCorrected, GriffinPacket joystick, float* thrustPtr) {
  Quaternion vect_axisX = {0.0f, 1.0f, 0.0f, 0.0f};
  Quaternion vect_axisZ = {0.0f, 0.0f, 0.0f, 1.0f};

  Quaternion vect_axisXimu = QuaternionRotateVector(imuCorrected, vect_axisX);
  float yawActual = atan2f(vect_axisXimu.y, vect_axisXimu.x);
  //Quaternion imuYaw = {cosf(yawActual/2.0f), 0.0f, 0.0f, sinf(yawActual/2.0f)};
  imuYaw.w = cosf(yawActual/2.0f);
  imuYaw.z = sinf(yawActual/2.0f);
  //Quaternion imuRollPitch;
  QuaternionsMultiply(&imuRollPitch, QuaternionConjugate(imuYaw), imuCorrected);
  Quaternion vect_axisZimuNoYaw = QuaternionRotateVector(imuRollPitch, vect_axisZ);
  float pitchActual = atan2f(vect_axisZimuNoYaw.x, vect_axisZimuNoYaw.z);
  float rollActual = -atan2f(vect_axisZimuNoYaw.y, vect_axisZimuNoYaw.z);
  // TODO: compensate thrust based on vect_axisZimuNoYaw.z component

  float rollJoy = joystick.leftRight, pitchJoy = joystick.upDown, yawJoy = joystick.padLeftRight;
  // Convert joystick values into angles
  yawJoy *= -PI / 32768.0f / 2.0f;
  rollJoy *= JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  pitchJoy *= JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;

  // Trim the joystick
  rollJoy += TRIM_ANGLE_PER_PRESS * (float)rightTrim;
  pitchJoy += TRIM_ANGLE_PER_PRESS * (float)forwardTrim;

  RollPitchYaw result;
  result.roll  = rollJoy  - rollActual;
  result.pitch = pitchJoy - pitchActual;
  result.yaw   = yawJoy   - yawActual;
  return result;
}

// Filter results from GetQuaternionError to improve PID step response
void LimitErrors(RollPitchYaw* errors) {
#ifdef LIMIT_PID_ERROR_YAW
  if (errors->yaw > LIMIT_PID_ERROR_YAW) errors->yaw = LIMIT_PID_ERROR_YAW;
  else if (errors->yaw < -LIMIT_PID_ERROR_YAW) errors->yaw = -LIMIT_PID_ERROR_YAW;
#endif
#ifdef LIMIT_PID_ERROR_PITCH
  if (errors->pitch > LIMIT_PID_ERROR_PITCH) errors->pitch = LIMIT_PID_ERROR_PITCH;
  else if (errors->pitch < -LIMIT_PID_ERROR_PITCH) errors->pitch = -LIMIT_PID_ERROR_PITCH;
#endif
#ifdef LIMIT_PID_ERROR_ROLL
  if (errors->roll > LIMIT_PID_ERROR_ROLL) errors->roll = LIMIT_PID_ERROR_ROLL;
  else if (errors->roll < -LIMIT_PID_ERROR_ROLL) errors->roll = -LIMIT_PID_ERROR_ROLL;
#endif
}

GyroData limitGyro(GyroData original) {
  GyroData result;
#ifdef LIMIT_GYRO_ERROR_ROLL
  if (original.x > LIMIT_GYRO_ERROR_ROLL) result.x = LIMIT_GYRO_ERROR_ROLL;
  else if (original.x < -LIMIT_GYRO_ERROR_ROLL) result.x = -LIMIT_GYRO_ERROR_ROLL;
  else result.x = original.x;
#else
  result.x = original.x;
#endif
#ifdef LIMIT_GYRO_ERROR_PITCH
  if (original.y > LIMIT_GYRO_ERROR_PITCH) result.y = LIMIT_GYRO_ERROR_PITCH;
  else if (original.y < -LIMIT_GYRO_ERROR_PITCH) result.y = -LIMIT_GYRO_ERROR_PITCH;
  else result.y = original.y;
#else
  result.y = original.y;
#endif
#ifdef LIMIT_GYRO_ERROR_YAW
  if (original.z > LIMIT_GYRO_ERROR_YAW) result.z = LIMIT_GYRO_ERROR_YAW;
  else if (original.z < -LIMIT_GYRO_ERROR_YAW) result.z = -LIMIT_GYRO_ERROR_YAW;
  else result.z = original.z;
#else
  result.z = original.z;
#endif
  return result;
}

/* from the point of view of the thrust vectors
+yaw <-----+
   +pitch   \
     /\      \
    CCW1
 CW0    CW2-> +roll
    CCW3
 */
// positive roll thrust will make roll tape go down, similar for rest
// Motor vectors which define which directions are which
float THRUST_VECTOR_ROLL[4] = {0.0f, -1.0f, 0.0f, 1.0f};
float THRUST_VECTOR_PITCH[4] = {-1.0f, 0.0f, 1.0f, 0.0f};
float THRUST_VECTOR_YAW[4] = {1.0f, -1.0f, 1.0f, -1.0f};
void PID(float* motorVals, RollPitchYaw rotations, GyroData gyroData, float thrust) {
  float rollVect[4], pitchVect[4], yawVect[4];
  // motorVals = Proportional commands
  VectorScale(rollVect, GAIN_PROPORTIONAL_ROLL * rotations.roll, THRUST_VECTOR_ROLL);
  VectorScale(pitchVect, GAIN_PROPORTIONAL_PITCH * rotations.pitch, THRUST_VECTOR_PITCH);
  VectorScale(yawVect, GAIN_PROPORTIONAL_YAW * rotations.yaw, THRUST_VECTOR_YAW);
  Vectors3Add(motorVals, rollVect, pitchVect, yawVect);

  gyroData = limitGyro(gyroData);
  // derivativeMVals = Derivative commands
  float derivativeMVals[4];
  VectorScale(rollVect, GAIN_DERIVATIVE_ROLL * gyroData.y, THRUST_VECTOR_ROLL);
  VectorScale(pitchVect, GAIN_DERIVATIVE_PITCH * gyroData.x, THRUST_VECTOR_PITCH);
  VectorScale(yawVect, GAIN_DERIVATIVE_YAW * gyroData.z, THRUST_VECTOR_YAW);
  Vectors3Add(derivativeMVals, rollVect, pitchVect, yawVect);
  
  // motorVals += derivativeMVals and thrust/average adjustment
  float average = 0.0f;
  for (int i = 0; i < 4; i++) {
    motorVals[i] += derivativeMVals[i];
    average += motorVals[i];
  }
  average /= 4.0f;
  VectorScalarAdd(motorVals, thrust - average, motorVals);
}

Quaternion Joystick2Quaternion(int16_t rollInt, int16_t pitchInt, int16_t yawInt) {
  float roll = rollInt, pitch = pitchInt, yaw = yawInt;
  // Convert joystick values into their corresponding individual rotation as a Quaternion
  yaw *= -PI / 32768.0f / 2.0f;
  roll *= JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  pitch *= JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  Quaternion yawQuat = {cosf(yaw), 0.0f, 0.0f, sinf(yaw)};
  Quaternion pitchQuat = {cosf(pitch), 0.0f, sinf(pitch), 0.0f};
  Quaternion rollQuat = {cosf(roll), sinf(roll), 0.0f, 0.0f};
  
  Quaternion rollPitch, earth2RollPitchTrim, earth2Desired;
  // Combine roll and pitch into one rotation
  QuaternionsMultiply(&rollPitch, pitchQuat, rollQuat);
  // TrimQuaternion = earth2Trimmed
  // If you reverse this multiply, trim will be from the frame of reference of your pre-trimmed desired rollPitch.
  // I think that makes less sense than doing the rollPitch from the frame of reference of the trimmed orientation, which is what this does.
  QuaternionsMultiply(&earth2RollPitchTrim, TrimQuaternion, rollPitch);
  // Yaw the whole trimmed roll/pitch to the desired orientation
  QuaternionsMultiply(&earth2Desired, yawQuat, earth2RollPitchTrim);
  return earth2Desired;
}

// Takes in a button press and applies trim to the trim quaternion
void JoystickApplyTrim(uint16_t button) {
  // The rotations for a single trim button press (set in the function that uses them
  static Quaternion QUAT_TRIM_RIGHT = {0.0f, 0.0f, 0.0f, 0.0f};
  static Quaternion QUAT_TRIM_DOWN  = {0.0f, 0.0f, 0.0f, 0.0f};
  static Quaternion QUAT_TRIM_LEFT  = {0.0f, 0.0f, 0.0f, 0.0f};
  static Quaternion QUAT_TRIM_UP    = {0.0f, 0.0f, 0.0f, 0.0f};
  static bool initDone = false;
  if (!initDone) {
    float w = cosf(TRIM_ANGLE_PER_PRESS / 2.0f);
    QUAT_TRIM_RIGHT.w = w;
    QUAT_TRIM_LEFT.w  = w;
    QUAT_TRIM_UP.w    = w;
    QUAT_TRIM_DOWN.w  = w;
    
    float xy = sinf(TRIM_ANGLE_PER_PRESS / 2.0f);
    QUAT_TRIM_RIGHT.x = xy;
    QUAT_TRIM_LEFT.x  = -xy;
    QUAT_TRIM_UP.y   = xy;
    QUAT_TRIM_DOWN.y = -xy;
    
    initDone = true;
  }
  
  switch (button) {
  case JOYSTICK_BUTTON_TRIM_RIGHT:
    QuaternionsMultiply(&TrimQuaternion, TrimQuaternion, QUAT_TRIM_RIGHT);
    rightTrim++;
    break;
  case JOYSTICK_BUTTON_TRIM_LEFT:
    QuaternionsMultiply(&TrimQuaternion, TrimQuaternion, QUAT_TRIM_LEFT);
    rightTrim--;
    break;
  case JOYSTICK_BUTTON_TRIM_UP:
    QuaternionsMultiply(&TrimQuaternion, TrimQuaternion, QUAT_TRIM_UP);
    forwardTrim++;
    break;
  case JOYSTICK_BUTTON_TRIM_DOWN:
    QuaternionsMultiply(&TrimQuaternion, TrimQuaternion, QUAT_TRIM_DOWN);
    forwardTrim--;
    break;
  }
}
