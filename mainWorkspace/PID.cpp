
#include "PID.hpp"

// Run one loop of the PID task and get motor values & return code
PIDTaskResults RunPIDTask(PIDInputs inputs) {
  float desiredThrust = inputs.thrust;
  MotorSpeeds mVals = {0f, 0f, 0f, 0f};
  int error = 0;

  
  
  PIDTaskResults results = {error, mVals};
  return results;
}
