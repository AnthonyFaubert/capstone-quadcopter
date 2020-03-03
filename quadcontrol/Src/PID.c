
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

// Rotates a 3D vector stored in a Quaternion struct by a quaternion and then returns the resulting 3D vector stored in a Quaternion struct.
Quaternion QuatRotateVector(Quaternion rotation, Quaternion vector) {
  Quaternion result, rotation_conj;
  QuaternionConjugate(&rotation_conj, rotation);
  QuaternionsMultiply(&result, rotation, vector);
  QuaternionsMultiply(&result, vector, rotation_conj);
  return result;
}

// Gives the changes in roll, pitch, and yaw required to get from the actual orientation to the desired orientation
// Goal is in two components: (roll/pitch), and yaw. roll/pitch is in the quadcopter frame and yaw is in the earth frame.
void GetQuaternionError(RollPitchYaw* result, Quaternion earth2Quadcopter, Quaternion rollPitchGoal, float earth2DesiredYaw_angle) {
  Quaternion axisX = {0.0f, 1.0f, 0.0f, 0.0f};
  Quaternion axisZ = {0.0f, 0.0f, 0.0f, 1.0f};

  Quaternion axisXQuadcopter = QuatRotateVector(earth2Quadcopter, axisX);
  float earth2Quadyaw_angle = atan2f(axisXQuadcopter.y, axisXQuadcopter.x); // this is atan2f(y=y, x=x), not atan2f(x=y, y=x)
  Quaternion earth2DesiredYaw = {cosf(earth2Quadyaw_angle/2.0f), 0.0f, 0.0f, sin(earth2Quadyaw_angle/2.0f)};

  // rotAx = [x=0, y=0, z=1]
  // W=cos(alpha/2), X=rotAx_x * sin(alpha/2), Y=rotAx_y * sin(alpha/2), Z=rotAx_z * sin(alpha/2)

  Quaternion earth2Desired;
  QuaternionsMultiply(&earth2Desired, earth2DesiredYaw, rollPitchGoal);

  Quaternion quadcopter2Desired, quadcopter2Earth;
  QuaternionConjugate(&quadcopter2Earth, earth2Quadcopter);
  QuaternionsMultiply(&quadcopter2Desired, earth2Desired, quadcopter2Earth);

  Quaternion axisXCorrected = QuatRotateVector(quadcopter2Desired, axisX);
  Quaternion axisZCorrected = QuatRotateVector(quadcopter2Desired, axisZ);

  result->roll = atan2f(axisZCorrected.x, axisZCorrected.z);
  result->pitch = atan2f(axisZCorrected.y, axisZCorrected.z);
  result->yaw = 0.0f; // FIXME
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
  roll *= JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  pitch *= -JOYSTICK_MAX_ANGLE / 32768.0f / 2.0f;
  Quaternion yawQuat = {cosf(yaw), 0.0f, 0.0f, sinf(yaw)};
  Quaternion rollQuat = {cosf(roll), 0.0f, sinf(roll), 0.0f};
  Quaternion pitchQuat = {cosf(pitch), sinf(pitch), 0.0f, 0.0f};
  
  // FIXME: fix comments
  //// Combine rotations; roll, pitch, then yaw
  //// Ideally roll and pitch at the same time and then yaw, but that's too complicated
  Quaternion tmpQuat;
  //QuaternionsMultiply(&tmpQuat, TrimQuaternion, yawQuat);
  QuaternionsMultiply(&tmpQuat, pitchQuat, TrimQuaternion);
  QuaternionsMultiply(joyCmdQuatPtr, rollQuat, tmpQuat);
  //QuaternionsMultiply(&tmpQuat, pitchQuat, rollQuat);
  //QuaternionsMultiply(joyCmdQuatPtr, yawQuat, tmpQuat);
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
    QUAT_TRIM_RIGHT.y = xy;
    QUAT_TRIM_LEFT.y  = -xy;
    QUAT_TRIM_DOWN.x = xy;
    QUAT_TRIM_UP.x   = -xy;
    
    initDone = true;
  }
  
  switch (button) {
  case JOYSTICK_BUTTON_TRIM_RIGHT: QuaternionsMultiply(&TrimQuaternion, TrimQuaternion, QUAT_TRIM_RIGHT);
      break;
  case JOYSTICK_BUTTON_TRIM_LEFT:  QuaternionsMultiply(&TrimQuaternion, TrimQuaternion, QUAT_TRIM_LEFT);
      break;
  case JOYSTICK_BUTTON_TRIM_UP:    QuaternionsMultiply(&TrimQuaternion, TrimQuaternion, QUAT_TRIM_UP);
      break;
  case JOYSTICK_BUTTON_TRIM_DOWN:  QuaternionsMultiply(&TrimQuaternion, TrimQuaternion, QUAT_TRIM_DOWN);
      break;
  }
}
