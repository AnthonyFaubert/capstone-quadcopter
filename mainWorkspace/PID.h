
#ifndef __PID_H__
#define __PID_H__

// A set of floats for each motor thrust/speed quantity with 0.0 = no thrust and 1.0 = max thrust
typedef struct {
  float frontLeft;
  float frontRight;
  float rearLeft;
  float rearRight;
} MotorSpeeds;

// The result returned from each call to the PID loop.
typedef struct {
  int errorCode; // 0 = no error, anything else = error
  MotorSpeeds motorSpeeds;
} PIDTaskResults;

#endif
