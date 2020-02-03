
//#include "main.h"
//#include "dma.h"
#include "i2c.h"
//#include "i2s.h"
//#include "spi.h"
#include "stdbool.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include "bno055.h"
#include "accel.h"
#include "quadcontrol.h"


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
        while (HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10) != HAL_OK) {};
 return ch;
}


uint8_t strBuf[500];

void usbprintln(const char* str) {
  uint16_t len = sprintf((char*) strBuf, "%s\n", str);
  CDC_Transmit_FS(strBuf, len);
}

bool checkButtonState(bool high) {
  if (high) {
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET;
  } else {
    return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET;
  }
}
void waitForButtonState(bool high, bool printPrompt) {
  for (int i = 0; !checkButtonState(high); i++) {
    if (printPrompt && (i % 10 == 0)) {
      usbprintln("Waiting for button press...");
    }
    HAL_Delay(100);
  }
}
void waitWithEStopCheck(int ms) {
  for (int i = 0; i < ms/10; i++) {
    if (checkButtonState(true)) {
      emergencyStop();
    }
    HAL_Delay(10);
  }
}

void SetPWM(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
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

void emergencyStop() {
  SetPWM(0, 0, 0, 0);
  usbprintln("EMERGENCY STOP ACTIVATED!");
  while (1);
}

// Main program entry point
void quadcontrol() {
  BNO055_Init_I2C(&hi2c1);
  
  usbprintln("Started.");
  waitForButtonState(true, true);
  HAL_Delay(100);
  waitForButtonState(false, true);
  
  usbprintln("Calibrating ESCs...");
  for (uint16_t pwm = 1000; pwm <= 2000; pwm += 200) {
    SetPWM(pwm, pwm, pwm, pwm);
    HAL_Delay(100);
  }
  SetPWM(1000, 1000, 1000, 1000);
  usbprintln("ESCs calibrated.");

  
  uint8_t imu_readings[IMU_NUMBER_OF_BYTES];
  int16_t accel_data[3];
  float acc_x, acc_y, acc_z;

  uint16_t ch2pwm = 1000;
  while (1) {
    waitWithEStopCheck(1000);
    SetPWM(1000, ch2pwm, 1500, 2000);
    ch2pwm += 200;
    if (ch2pwm > 2000) ch2pwm = 1000;
    
    GetAccelData(&hi2c1, (uint8_t*)imu_readings);
    accel_data[0] = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);      // Turn the MSB and LSB into a signed 16-bit value
    accel_data[1] = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
    accel_data[2] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
    acc_x = ((float)(accel_data[0]))/100.0f; //m/s2
    acc_y = ((float)(accel_data[1]))/100.0f;
    acc_z = ((float)(accel_data[2]))/100.0f;
    int len = sprintf((char*) strBuf, "X: %.2f Y: %.2f Z: %.2f\r\n", acc_x, acc_y, acc_z);
    CDC_Transmit_FS(strBuf, len);
  }
}
