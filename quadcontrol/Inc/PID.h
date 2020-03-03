
#ifndef __PID_H__
#define __PID_H__

#include "stdbool.h"
#include "stdint.h"
#include "VectQuatMath.h"

// Set max bank angle to 22.5 degrees
#define JOYSTICK_MAX_ANGLE (PI / 8.0f)

#define JOYSTICK_BUTTON_TRIM_RIGHT 2
#define JOYSTICK_BUTTON_TRIM_LEFT 3
#define JOYSTICK_BUTTON_TRIM_DOWN 1
#define JOYSTICK_BUTTON_TRIM_UP 4
#define TRIM_ANGLE_PER_PRESS (1.0f * PI / 180.0f)

// Used by LimitErrors()    (pi/8) = 22.5 degrees
//#define LIMIT_PID_ERROR_YAW (PI / 8.0f)
//#define LIMIT_PID_ERROR_PITCH (PI / 8.0f)
//#define LIMIT_PID_ERROR_ROLL (PI / 8.0f)

// Used internally to limit gyro values before using them for PID. Same units as gyro (degs/sec)
//#define LIMIT_GYRO_ERROR_ROLL 50.0f
//#define LIMIT_GYRO_ERROR_PITCH 50.0f
//#define LIMIT_GYRO_ERROR_YAW 50.0f

// Gain values for PID

#define GAIN_PROPORTIONAL_ROLL  -0.24f
#define GAIN_PROPORTIONAL_PITCH -0.24f
#define GAIN_PROPORTIONAL_YAW   0.0f
#define GAIN_DERIVATIVE_ROLL   -0.0013f
#define GAIN_DERIVATIVE_PITCH  -0.0013f
#define GAIN_DERIVATIVE_YAW    0.0f

#define ORIENTATION_CORRECTION_QUATERNION {0.9063077870366499f, 0.0f, 0.0f, 0.42261826174069944f}
#define GYRO_CORRECTION_COSINE 0.6427876096865394f
#define GYRO_CORRECTION_SINE 0.766044443118978f


typedef struct {
  float x, y, z;
} GyroData;

// A set of angle differences between a current heading and a desired one.
typedef struct {
  float roll, pitch, yaw;
} RollPitchYaw;

// Trim quaternion maintained by JoystickApplyTrim()
extern Quaternion TrimQuaternion;

// Apply IMU physical orientation mismatch correction factor 
void ApplyOrientationCorrection(Quaternion* orientation);
// Apply IMU physical orientation mismatch correction factor to gyro data
void ApplyGyroCorrection(GyroData* gyroData);

// Gives the changes in roll, pitch, and yaw required to get from the actual orientation to the desired orientation
RollPitchYaw GetQuaternionError(Quaternion actual, Quaternion desired);
// Filter results from GetQuaternionError to improve PID step response
void LimitErrors(RollPitchYaw* quatErrors);
// Takes in values from GetQuaternionError(), gyroscope data, and a thrust value to produce the values which should be applied to the motors
void PID(float* motorVals, RollPitchYaw rotations, GyroData gyroData, float thrust);
// Takes in raw values from the joystick and returns a Quaternion representing the desired orientation
Quaternion Joystick2Quaternion(int16_t rollInt, int16_t pitchInt, int16_t yawInt);
// Takes in a button press and applies trim to TrimQuaternion
void JoystickApplyTrim(uint16_t button);

#endif
