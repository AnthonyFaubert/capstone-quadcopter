
#ifndef __IMU_H__
#define __IMU_H__

#include "stdbool.h"
#include "stdint.h"
#include "VectQuatMath.h" // for Quaternion typedef
#include "PID.h" // for GyroData typedef

#define ACC_MEASURE_PERIOD 91 // 20 [ms] => 50Hz; 10 => 100Hz (91 oli enne seal)
#define	IMU_NUMBER_OF_BYTES 18 // Number of bytes to read from IMU register

extern uint8_t imu_readings[IMU_NUMBER_OF_BYTES];

void IMUInit();
bool IMUGetOrientation(Quaternion* orientation);
bool IMUGetGyro(GyroData* gyroData);

#endif
