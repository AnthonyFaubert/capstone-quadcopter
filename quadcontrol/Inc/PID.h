
#ifndef __PID_H__
#define __PID_H__

#include "stdbool.h"
#include "stdint.h"
#include "VectQuatMath.h"
#include "Quadcontrol.h"

// Set max bank angle to 22.5 degrees
#define JOYSTICK_MAX_ANGLE (PI / 8.0f)

#define JOYSTICK_BUTTON_TRIM_RIGHT 2
#define JOYSTICK_BUTTON_TRIM_LEFT 3
#define JOYSTICK_BUTTON_TRIM_DOWN 1
#define JOYSTICK_BUTTON_TRIM_UP 4
#define TRIM_ANGLE_PER_PRESS (1.0f * PI / 180.0f)

// Used by LimitErrors()    (pi/8) = 22.5 degrees
#define LIMIT_PID_ERROR_YAW (PI / 6.0f)
//#define LIMIT_PID_ERROR_PITCH (PI / 8.0f)
//#define LIMIT_PID_ERROR_ROLL (PI / 8.0f)

// Used internally to limit gyro values before using them for PID. Same units as gyro (degs/sec)
//#define LIMIT_GYRO_ERROR_ROLL 50.0f
//#define LIMIT_GYRO_ERROR_PITCH 50.0f
//#define LIMIT_GYRO_ERROR_YAW 50.0f

// Gain values for PID, all test comments done on vertical string test
#define GAIN_PROPORTIONAL_ROLL  0.09f//0.18 was roughly where it started to oscillate too heavily
#define GAIN_PROPORTIONAL_PITCH 0.12f//0.16 was roughly where it started to oscillate too heavily
#define GAIN_PROPORTIONAL_YAW   0.25f//0.4f was where it oscillated
#define GAIN_DERIVATIVE_ROLL   -0.0008f//0.0018 was a bit over where it would begin oscillating, 0.0015 was below
#define GAIN_DERIVATIVE_PITCH  0.00065f//0.0013 was where it began oscillating
#define GAIN_DERIVATIVE_YAW    -0.008f//-0.004f

#define ORIENTATION_CORRECTION_QUATERNION {0.9063077870366499f, 0.0f, 0.0f, 0.42261826174069944f}
#define GYRO_CORRECTION_COSINE 0.66262004821f
#define GYRO_CORRECTION_SINE 0.74895572078f


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

// Gives rotation required to get from the actual orientation to the desired orientation
Quaternion GetQuaternionError(Quaternion actual, Quaternion desired);
// Converts a quaternion rotation to an Euler rotation representation
RollPitchYaw Quaternion2Euler(Quaternion q);
// New control system takes in corrected IMU, joystick values, and mutable thrust and produces proportional errors
RollPitchYaw NewControl(Quaternion imuCorrected, GriffinPacket joystick, float* thrustPtr);
extern Quaternion imuRollPitch, imuYaw; // temporary globals for printouts
// Filter results from GetQuaternionError to improve PID step response
void LimitErrors(RollPitchYaw* quatErrors);
// Takes in values from GetQuaternionError(), gyroscope data, and a thrust value to produce the values which should be applied to the motors
void PID(float* motorVals, RollPitchYaw rotations, GyroData gyroData, float thrust);
// Takes in raw values from the joystick and returns a Quaternion representing the desired orientation
Quaternion Joystick2Quaternion(int16_t rollInt, int16_t pitchInt, int16_t yawInt);
// Takes in a button press and applies trim to TrimQuaternion
void JoystickApplyTrim(uint16_t button);

#endif
