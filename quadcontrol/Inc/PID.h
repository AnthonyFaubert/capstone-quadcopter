
#ifndef __PID_H__
#define __PID_H__

#include "VectQuatMath.h"

// Set max bank angle to 45 degrees
#define JOYSTICK_MAX_ANGLE (PI / 4.0f)

#define JOYSTICK_BUTTON_TRIM_RIGHT 2
#define JOYSTICK_BUTTON_TRIM_LEFT 3
#define JOYSTICK_BUTTON_TRIM_DOWN 1
#define JOYSTICK_BUTTON_TRIM_UP 4
#define TRIM_ANGLE_PER_PRESS 1.0f * PI / 180.0f

typedef struct {
  float x, y, z;
} GyroData;

// A set of angle differences between a current heading and a desired one.
typedef struct {
  float roll, pitch, yaw;
} RollPitchYaw;

// Trim quaternion maintained by JoystickApplyTrim()
extern Quaternion TrimQuaternion;

// Gives the changes in roll, pitch, and yaw required to get from the actual orientation to the desired orientation
void GetQuaternionError(RollPitchYaw* result, Quaternion actual, Quaternion desired);
// Takes in values from GetQuaternionError(), gyroscope data, and a thrust value to produce the values which should be applied to the motors
void PID(float* motorVals, RollPitchYaw rotations, GyroData gyroData, float thrust);
// Takes in raw values from the joystick and produces a Quaternion representing the desired orientation
void Joystick2Quaternion(Quaternion* joystick, uint16_t roll, uint16_t pitch, uint16_t yaw);
// Takes in a button press and applies trim to TrimQuaternion
void JoystickApplyTrim(uint16_t button);

#endif