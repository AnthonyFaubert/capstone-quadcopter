
#include "PID.h"
#include "VectQuatMath.h"
#include "math.h"

Quaternion TrimQuaternion = {1.0f, 0.0f, 0.0f, 0.0f};

void ApplyOrientationCorrection(Quaternion* orientation) {
  static Quaternion orientationCorrection = ORIENTATION_CORRECTION_QUATERNION;
  QuaternionsMultiply(orientation, *orientation, orientationCorrection);
  //QuaternionsMultiply(orientation, orientationCorrection, *orientation);
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

/* from the point of view of positive gain causing the roll/pitch/yaw
+yaw <-----+
   +pitch   \
     /\      \
    CCW1
 CW0    CW3-> +roll
    CCW2
 */
// positive roll thrust will make roll tape go down, similar for rest
// Motor vectors which define which directions are which
float THRUST_VECTOR_ROLL[4] = {1.0f, 0.0f, -1.0f, 0.0f};
float THRUST_VECTOR_PITCH[4] = {0.0f, -1.0f, 0.0f, 1.0f};
float THRUST_VECTOR_YAW[4] = {1.0f, -1.0f, 1.0f, -1.0f};
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
    QUAT_TRIM_RIGHT.x = xy;
    QUAT_TRIM_LEFT.x  = -xy;
    QUAT_TRIM_DOWN.y = xy;
    QUAT_TRIM_UP.y   = -xy;
    
    initDone = true;
  }
  
  switch (button) {
  case JOYSTICK_BUTTON_TRIM_RIGHT: QuaternionsMultiply(&TrimQuaternion, QUAT_TRIM_RIGHT, TrimQuaternion);
  case JOYSTICK_BUTTON_TRIM_LEFT:  QuaternionsMultiply(&TrimQuaternion, QUAT_TRIM_LEFT, TrimQuaternion);
  case JOYSTICK_BUTTON_TRIM_UP:    QuaternionsMultiply(&TrimQuaternion, QUAT_TRIM_UP, TrimQuaternion);
  case JOYSTICK_BUTTON_TRIM_DOWN:  QuaternionsMultiply(&TrimQuaternion, QUAT_TRIM_DOWN, TrimQuaternion);
  }
}
