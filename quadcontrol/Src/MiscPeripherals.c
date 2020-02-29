
#include "MiscPeripherals.h"
#include "stdbool.h"
#include "tim.h"
#include "gpio.h"
#include "VectQuatMath.h"
#include "math.h" // for INFINITY

static bool calibrated = false;

void setIMUResetState(bool reset) {
	HAL_GPIO_WritePin(GPIOE, IMU_RST_L_Pin, reset ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// Returns whether or not the blue button state matches the given state (pressed=true)
bool CheckButtonState(bool high) {
  if (high) {
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET;
  } else {
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
  }
}

static void setESCs(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
  
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  sConfigOC.Pulse = ch1;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch3;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch4;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

// Returns error type: 0 = no error, 1 = thrust request not honored, -1 = non-linearity fatal error
uint8_t SetMotors(float* motorVals) {
  if (!calibrated) return SETMOTORS_FAILED_NOTCALIBRATED;
  uint8_t errors = SETMOTORS_OK;

  // Find largest/smallest values and copy motor values, clipping values that are out of bounds
  float tmp[4];
  float largest = -INFINITY;
  float smallest = INFINITY;
  for (int i = 0; i < 4; i++) {
    if (motorVals[i] > largest) largest = motorVals[i];
    if (motorVals[i] < smallest) smallest = motorVals[i];

    // copy, clip, and detect errors
    if (motorVals[i] < 0.0f) {
      tmp[i] = 0.0f;
      errors |= SETMOTORS_CLIPPED_0 << i;
    } else if (1.0f < motorVals[i]) {
      tmp[i] = 1.0f;
      errors |= SETMOTORS_CLIPPED_0 << i;
    } else {
      tmp[i] = motorVals[i];
    }
  }

  if ((largest - smallest) > 1.0f) errors |= SETMOTORS_NOT_LINEARIZABLE;

  /* old shifting code, potentially masks PID problems
  if (errors) {
    if ((largest - smallest) > 1.0f) {
      setESCs(1000, 1000, 1000, 1000);
      return errors;
    } else {
      if (largest > 1.0f) {
        VectorScalarAdd(motorVals, 1.0f - largest, motorVals);
      } else {
        VectorScalarAdd(motorVals, -smallest, motorVals);
      }
    }
  }
  */

  // Convert motor values [0.0f, 1.0f] to PWM values
  uint16_t thrusts[4];
  for (int i = 0; i < 4; i++) {
    tmp[i] *= (float) (MAX_THRUST - MIN_THRUST);
    thrusts[i] = ((uint16_t) tmp[i]) + MIN_THRUST;
  }
  //PRINTF("MVsRaw: %d, %d, %d, %d\n", thrusts[MOTOR_CHANNEL_MAPPING[0]], thrusts[MOTOR_CHANNEL_MAPPING[1]], thrusts[MOTOR_CHANNEL_MAPPING[2]], thrusts[MOTOR_CHANNEL_MAPPING[3]]);
  setESCs(thrusts[MOTORMAP_CH1_MOTOR], thrusts[MOTORMAP_CH2_MOTOR], thrusts[MOTORMAP_CH3_MOTOR], thrusts[MOTORMAP_CH4_MOTOR]);
  return errors;
}

void EmergencyShutoff() {
  setESCs(0, 0, 0, 0);
}

#ifndef CALIBRATE_ESCS_WAIT_MACRO
#define CALIBRATE_ESCS_WAIT_MACRO(ms) HAL_Delay(ms)
#endif
void CalibrateESCs() {
  for (uint16_t pwm = 1000; pwm <= 2000; pwm += 200) {
    setESCs(pwm, pwm, pwm, pwm);
    CALIBRATE_ESCS_WAIT_MACRO(100);
  }
  setESCs(1000, 1000, 1000, 1000);
  CALIBRATE_ESCS_WAIT_MACRO(1000);
  calibrated = true;
}
