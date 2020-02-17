
#ifndef __MISC_PERIPHERALS_H__
#define __MISC_PERIPHERALS_H__

const char MOTOR_CHANNEL_MAPPING[4] = {2, 3, 0, 1}; // with GND down and SIG up, left to right, the slots are: 2, 0, 1, 3
uint16_t MIN_THRUSTS[4] = {1050, 1050, 1050, 1050};
uint16_t MAX_THRUSTS[4] = {1600, 1600, 1600, 1600};

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
void SetMotors(float* motorValues);

// Turns off motor PWMs
void EmergencyShutoff();

#endif
