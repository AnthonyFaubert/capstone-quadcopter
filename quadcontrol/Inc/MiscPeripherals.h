
#ifndef __MISC_PERIPHERALS_H__
#define __MISC_PERIPHERALS_H__

#include "stdbool.h"
#include "stdint.h"

#define MIN_THRUST 1050
#define MAX_THRUST 2000

// Mappings from PWM channels to indexes in the motor value arrays
#define MOTORMAP_CH1_MOTOR 1
#define MOTORMAP_CH2_MOTOR 2
#define MOTORMAP_CH3_MOTOR 0
#define MOTORMAP_CH4_MOTOR 3

// setIMUResetState(true) turns on IMU hard reset, false to turn off
void setIMUResetState(bool reset);

// Returns whether or not the blue button state matches the given state (pressed=true)
bool CheckButtonState(bool high);

// Calibrate the ESCs so that we can use them. Takes 1.5 seconds.
// Define CALIBRATE_ESCS_WAIT_MACRO(milliseconds) to do something while it waits.
void CalibrateESCs();

// Possible values returned by SetMotors()
// Motor values had to be shifted into valid range; total thrust will be different
#define SETMOTORS_THRUST_DENIED 1
// ERROR: motor values are non-linear; didn't set PWM values
#define SETMOTORS_FAILED_NONLINEAR -1
// ERROR: you never called CalibrateESCs()
#define SETMOTORS_FAILED_NOTCALIBRATED -2
// Everything fine
#define SETMOTORS_OK 0
// Sets the motor values to those specified with checking/adjusting to fit the valid range [0.0, 1.0]
int SetMotors(float* motorValues);

// Turns off motor PWMs
void EmergencyShutoff();

#endif
