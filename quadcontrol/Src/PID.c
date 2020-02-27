
#include "PID.h"
#include "VectQuatMath.h"
#include "math.h"

Quaternion TrimQuaternion = {1.0f, 0.0f, 0.0f, 0.0f};

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

// Gives the changes in roll, pitch, and yaw required to get from the actual orientation to the desired orientation
void GetQuaternionError(RollPitchYaw* result, Quaternion actual, Quaternion desired) {
  // Rotate actual by the opposite of desired, then desired is <1,0,0,0> (rotate by q is q*a*conj(q))
  // Compute the rotation from actual to desired (invert actual to get back to straight and then go to desired)
  Quaternion correctionRotation;
  QuaternionConjugate(&actual, actual);
  QuaternionsMultiply(&correctionRotation, actual, desired); // Overall rotation of (q*p) is rotate by p and then q

  // Convert the quaternion back into something we understand by using it to rotate <0,0,1> (for roll & pitch) and <1,0,0> (for yaw)
  Quaternion tmpA, tmpB, rotatedVector;
  Quaternion straight = {0.0f, 0.0f, 0.0f, 1.0f}; // straight = <0,0,1>
  QuaternionsMultiply(&tmpA, correctionRotation, straight);
  straight.x = 1.0f; straight.z = 0.0f; // straight = <1,0,0>
  QuaternionsMultiply(&tmpB, correctionRotation, straight);
  QuaternionConjugate(&correctionRotation, correctionRotation);
  QuaternionsMultiply(&rotatedVector, tmpA, correctionRotation);
  // Overall, rotatedVector = correctionRotation * <0,0,0,1> * correctionRotation^-1 = <0,0,1> rotated by correction
  result->roll = atan2f(rotatedVector.x, rotatedVector.z);
  result->pitch = atan2f(rotatedVector.y, rotatedVector.z);
  QuaternionsMultiply(&rotatedVector, tmpB, correctionRotation);
  // Overall, rotatedVector = correctionRotation * <0,1,0,0> * correctionRotation^-1 = <1,0,0> rotated by correction
  result->yaw = atan2f(rotatedVector.y, rotatedVector.x);
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
#endif
#ifdef LIMIT_GYRO_ERROR_PITCH
  if (original.y > LIMIT_GYRO_ERROR_PITCH) result.y = LIMIT_GYRO_ERROR_PITCH;
  else if (original.y < -LIMIT_GYRO_ERROR_PITCH) result.y = -LIMIT_GYRO_ERROR_PITCH;
  else result.y = original.y;
#endif
#ifdef LIMIT_GYRO_ERROR_YAW
  if (original.z > LIMIT_GYRO_ERROR_YAW) result.z = LIMIT_GYRO_ERROR_YAW;
  else if (original.z < -LIMIT_GYRO_ERROR_YAW) result.z = -LIMIT_GYRO_ERROR_YAW;
  else result.z = original.z;
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
float THRUST_VECTOR_ROLL[4] = {1.0f, 0.0f, -1.0f, 0.0f};
float THRUST_VECTOR_PITCH[4] = {0.0f, -1.0f, 0.0f, 1.0f};
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
  VectorScale(rollVect, GAIN_DERIVATIVE_ROLL * gyroData.x, THRUST_VECTOR_ROLL);
  VectorScale(pitchVect, GAIN_DERIVATIVE_PITCH * gyroData.y, THRUST_VECTOR_PITCH);
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

void Joystick2Quaternion(Quaternion* joyCmdQuatPtr, int16_t rollInt, int16_t pitchInt, int16_t yawInt) {
  float roll = rollInt;
  float pitch = pitchInt;
  float yaw = yawInt;
  yaw *= -PI / 32768.0f / 2.0f;
  roll *= -JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  pitch *= JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  Quaternion yawQuat = {cosf(yaw), 0.0f, 0.0f, sinf(yaw)};
  Quaternion rollQuat = {cosf(roll), 0.0f, sinf(roll), 0.0f};
  Quaternion pitchQuat = {cosf(pitch), sinf(pitch), 0.0f, 0.0f};
  
  // Combine rotations; yaw first, then roll, and finally pitch
  Quaternion tmpQuat;
  QuaternionsMultiply(&tmpQuat, rollQuat, yawQuat);
  QuaternionsMultiply(joyCmdQuatPtr, pitchQuat, tmpQuat);
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
    QUAT_TRIM_RIGHT.y = -xy;
    QUAT_TRIM_LEFT.y  = xy;
    QUAT_TRIM_DOWN.x = -xy;
    QUAT_TRIM_UP.x   = xy;
    
    initDone = true;
  }
  
  switch (button) {
  case JOYSTICK_BUTTON_TRIM_RIGHT: QuaternionsMultiply(&TrimQuaternion, QUAT_TRIM_RIGHT, TrimQuaternion);
  case JOYSTICK_BUTTON_TRIM_LEFT:  QuaternionsMultiply(&TrimQuaternion, QUAT_TRIM_LEFT, TrimQuaternion);
  case JOYSTICK_BUTTON_TRIM_UP:    QuaternionsMultiply(&TrimQuaternion, QUAT_TRIM_UP, TrimQuaternion);
  case JOYSTICK_BUTTON_TRIM_DOWN:  QuaternionsMultiply(&TrimQuaternion, QUAT_TRIM_DOWN, TrimQuaternion);
  }
}
