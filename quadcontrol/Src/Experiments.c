
#include "Experiments.h"
#include "Quadcontrol.h"
#include "math.h"
#include "VectQuatMath.h" // for PI

void experiment_SineWavePitch(uint32_t logTimestamp, float* mVals, float* thrust) {
  const float BASELINE_THRUST = 0.3f;
  const float RAMP_TIME_MS = 1000.0f;
  const uint32_t EXPERIMENT_START_MS = 5000; // must be more than ramp time
  const uint32_t EXPERIMENT_DURATION_MS = 14000;
  const float OMEGA = 2*PI; // 1 Hz
  const float AMPLITUDE = 0.1f;
  
  if (logTimestamp != 0xFFFFFFFF) { // start experiment 5 seconds into logging
    if (uwTick > logTimestamp+EXPERIMENT_START_MS+EXPERIMENT_DURATION_MS) { // exp. end
      emergencyStop();
    } else if (uwTick > logTimestamp+EXPERIMENT_START_MS) { // exp. start
      *thrust = BASELINE_THRUST;
      for (int i = 0; i < 4; i++) mVals[i] = *thrust;

      float time = uwTick - (logTimestamp + EXPERIMENT_START_MS);
      time /= 1000.0f; // ms -> sec
      float cmd = AMPLITUDE/2.0f * sin(OMEGA * time);
      mVals[1] -= cmd;
      mVals[3] += cmd;
    } else {
      *thrust = uwTick - logTimestamp;
      *thrust *= BASELINE_THRUST / RAMP_TIME_MS;
      if (*thrust > BASELINE_THRUST) *thrust = BASELINE_THRUST;
      for (int i = 0; i < 4; i++) mVals[i] = *thrust;
    }
  } else {
    for (int i = 0; i < 4; i++) mVals[i] = 0.0f;
  }
}

void experiment_SingleStepPitch(uint32_t logTimestamp, float* mVals, float* thrust) {
  const float BASELINE_THRUST = 0.3f;
  const float RAMP_TIME_MS = 2000.0f;
  const uint32_t EXPERIMENT_START_MS = 8000; // must be more than ramp time
  const uint32_t EXPERIMENT_DURATION_MS = 2000;
  const float MOTOR_COMMAND = 0.05f;
  
  if (logTimestamp != 0xFFFFFFFF) { // start experiment 5 seconds into logging
    if (uwTick > logTimestamp+EXPERIMENT_START_MS+EXPERIMENT_DURATION_MS) { // exp. end
      emergencyStop();
    } else if (uwTick > logTimestamp+EXPERIMENT_START_MS) { // exp. start
      *thrust = BASELINE_THRUST;
      for (int i = 0; i < 4; i++) mVals[i] = *thrust;
      
      mVals[1] -= MOTOR_COMMAND;
      mVals[3] += MOTOR_COMMAND;
    } else {
      *thrust = uwTick - logTimestamp;
      *thrust *= BASELINE_THRUST / RAMP_TIME_MS;
      if (*thrust > BASELINE_THRUST) *thrust = BASELINE_THRUST;
      for (int i = 0; i < 4; i++) mVals[i] = *thrust;
    }
  } else {
    for (int i = 0; i < 4; i++) mVals[i] = 0.0f;
  }
}

void experiment_CheckMotorMap(uint32_t logTimestamp, float* mVals) {
  if (logTimestamp != 0xFFFFFFFF) { // start experiment 5 seconds into logging
    for (int i = 0; i < 4; i++) mVals[i] = 0.0f;
    if      (uwTick > logTimestamp+13000) emergencyStop();
    else if (uwTick > logTimestamp+10000) mVals[3] = 0.2f;
    else if (uwTick > logTimestamp+ 7000) mVals[2] = 0.2f;
    else if (uwTick > logTimestamp+ 4000) mVals[1] = 0.2f;
    else if (uwTick > logTimestamp+ 1000) mVals[0] = 0.2f;
  } else {
    for (int i = 0; i < 4; i++) mVals[i] = 0.0f;
  }
}
