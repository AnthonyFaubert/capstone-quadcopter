
#ifndef __PID_H__
#define __PID_H__

#include "stdbool.h"
#include "stdint.h"
#include "VectQuatMath.h"

// Set max bank angle to 45 degrees
#define JOYSTICK_MAX_ANGLE (PI / 4.0f)

#define JOYSTICK_BUTTON_TRIM_RIGHT 2
#define JOYSTICK_BUTTON_TRIM_LEFT 3
#define JOYSTICK_BUTTON_TRIM_DOWN 1
#define JOYSTICK_BUTTON_TRIM_UP 4
#define TRIM_ANGLE_PER_PRESS 1.0f * PI / 180.0f

// Used by LimitErrors()    (pi/8) = 22.5 degrees
#define LIMIT_PID_ERROR_YAW (PI / 8.0f)
//#define LIMIT_PID_ERROR_PITCH
//#define LIMIT_PID_ERROR_ROLL

// Gain values for PID
#define GAIN_PROPORTIONAL_ROLL  0.08f
#define GAIN_PROPORTIONAL_PITCH 0.08f
#define GAIN_PROPORTIONAL_YAW   0.05f
#define GAIN_DERIVATIVE_ROLL   0.0015f
#define GAIN_DERIVATIVE_PITCH  0.0015f
#define GAIN_DERIVATIVE_YAW    0.0007f

#define ORIENTATION_CORRECTION_QUATERNION {0.64278760968f, 0.0f, 0.0f, 0.76604444311f}


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

// Gives the changes in roll, pitch, and yaw required to get from the actual orientation to the desired orientation
void GetQuaternionError(RollPitchYaw* result, Quaternion actual, Quaternion desired);
// Filter results from GetQuaternionError to improve PID step response
void LimitErrors(RollPitchYaw* quatErrors);
// Takes in values from GetQuaternionError(), gyroscope data, and a thrust value to produce the values which should be applied to the motors
void PID(float* motorVals, RollPitchYaw rotations, GyroData gyroData, float thrust);
// Takes in raw values from the joystick and produces a Quaternion representing the desired orientation
void Joystick2Quaternion(Quaternion* joystick, int16_t roll, int16_t pitch, int16_t yaw);
// Takes in a button press and applies trim to TrimQuaternion
void JoystickApplyTrim(uint16_t button);

#endif
