
#include "MiscPeripherals.h"
#include "stdbool.h"
#include "tim.h"
#include "gpio.h"
#include "VectQuatMath.h"

static bool calibrated = false;

// Returns whether or not the blue button state matches the given state (pressed=true)
bool CheckButtonState(bool high) {
  if (high) {
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET;
  } else {
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
  }
}

static void setESCs(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
  
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  sConfigOC.Pulse = ch1;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch2;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch3;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.Pulse = ch4;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

// Returns error type: 0 = no error, 1 = thrust request not honored, -1 = non-linearity fatal error
int SetMotors(float* motorVals) {
  if (!calibrated) return SETMOTORS_FAILED_NOTCALIBRATED;
  int error = -1;
  float largest = -INFINITY;
  float smallest = INFINITY;
  for (int i = 0; i < 4; i++) {
    if (motorVals[i] > largest) largest = motorVals[i];
    if (motorVals[i] < smallest) smallest = motorVals[i];
    
    if (motorVals[i] < 0.0f || 1.0f < motorVals[i]) {
      error = i;
    }
  }
  if (error != -1) {
    if ((largest - smallest) < 1.0f) {
      if (largest > 1.0f) {
        addScalar(motorVals, 1.0f - largest, motorVals);
      } else {
        addScalar(motorVals, -smallest, motorVals);
      }
    } else {
      SetPWM(1000, 1000, 1000, 1000);
      return SETMOTORS_FAILED_NONLINEAR;
    }
  }

  float tmp[4];
  uint16_t thrusts[4];
  for (int i = 0; i < 4; i++) {
    tmp[i] = motorVals[i] * (float) (MAX_THRUSTS[i] - MIN_THRUSTS[i]);
    thrusts[i] = ((uint16_t) tmp[i]) + MIN_THRUSTS[i];
  }
  //PRINTF("MVsRaw: %d, %d, %d, %d\n", thrusts[MOTOR_CHANNEL_MAPPING[0]], thrusts[MOTOR_CHANNEL_MAPPING[1]], thrusts[MOTOR_CHANNEL_MAPPING[2]], thrusts[MOTOR_CHANNEL_MAPPING[3]]);
  setPWM(thrusts[MOTOR_CHANNEL_MAPPING[0]], thrusts[MOTOR_CHANNEL_MAPPING[1]], thrusts[MOTOR_CHANNEL_MAPPING[2]], thrusts[MOTOR_CHANNEL_MAPPING[3]]);
  return (error != -1) ? SETMOTORS_THRUST_DENIED : SETMOTORS_OK;
}

void EmergencyShutoff() {
  setESCs(0, 0, 0, 0);
}

#ifndef CALIBRATE_ESCS_WAIT_MACRO
#define CALIBRATE_ESCS_WAIT_MACRO(ms) HAL_Delay(ms)
#endif
void CalibrateESCs() {
  for (uint16_t pwm = 1000; pwm <= 2000; pwm += 200) {
    SetPWM(pwm, pwm, pwm, pwm);
    CALIBRATE_ESCS_WAIT_MACRO(100);
  }
  SetPWM(1000, 1000, 1000, 1000);
  CALIBRATE_ESCS_WAIT_MACRO(1000);
  calibrated = true;
}
