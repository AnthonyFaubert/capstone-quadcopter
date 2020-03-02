
#ifndef __EXPERIMENTS_H__
#define __EXPERIMENTS_H__

#include "stdint.h"

// For experiments to e-stop when done
extern void emergencyStop();

// For confirming the motor mapping is correct.
// Turns on the motors at low values each for 2.5 seconds in a pattern (motor 0, then 1, 2, & 3).
void experiment_CheckMotorMap(uint32_t logTimestamp, float* mVals);


// These experiments require a swing jig //

// For determining good initial PD gains.
// Turns on the motors to a baseline level, waits for the operator to stop any swinging, then applies a sine wave command for several seconds.
void experiment_SineWavePitch(uint32_t logTimestamp, float* mVals, float* thrust);
// For determining the feedback delay.
// Turns on the motors to a baseline level, waits for the operator to stop any swinging, then does a small-ish pitch command.
// Plot mVals and gyroPitch on the same graph and measure the difference between the mVals changing and the gyroPitch responding to the command. That difference is the feedback delay.
void experiment_SingleStepPitch(uint32_t logTimestamp, float* mVals, float* thrust);

#endif
