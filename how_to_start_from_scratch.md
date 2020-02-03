STM32CubeMX -> File > New Project
select boards > STM32F4DISCOVERY
Start project ("Yes" to init peripherals with default modes)
Timers > TIM3 {
clk src internal
channel 1 pwm gen ch1
channel 2 pwm gen ch2
channel 3 pwm gen ch3
channel 4 pwm gen ch4
parameters:
prescaler 83
period 9999
clk div /4
}
Connectivity > USART2 {
mode async
}
Connectivity > USB_OTG_FS {
DISABLE "Activate_VBUS"
mode Device_Only
}
Middleware > USB_DEVICE {
Class for FS IP = communication device class (virtual port com)
}

/* BNO-specific config */
Connectivity > I2C1 {
DMA Settings tab > Add
 - I2C1_RX (DMA1 Stream 0)
 DMA Settings tab > Add
 - I2C1_TX (DMA1 Stream 6)
}
/////////////////////////


Project Manager tab {
proj location = .../capstone-quadcopter/quadcontrol
proj name = quadcontrol
min heap size = 0x1000
min stack size = 0x500
Code generator left tab > turn on "Generate peripheral initialization as a pair of '.c/.h' files per peripheral"
Advanced settings left tab > make sure USB_DEVICE IP has "Visibility (Static)" set to true
}

CTRL-s ("Yes" to download firmware, if applicable)

click GENERATE CODE
open project button

// Set target microcontroller, if not already set
Right click project in the Files pane on the left > Options...
General options tab, set Processor variant to "Device" and "STM32F407VG"

// Now do the true scratch section or the repo one
/* BNO files (true scratch) */
From here: https://gitlab.pld.ttu.ee/iotcenter/bno055/blob/master/bno055
download Src/accel.c Inc/{accel.h,bno055.h} and put them in the same spots in your IAR project.
Open IAR, right-click the (project)/Application/User group and click Add. Select accel.c (no action needed for headers in Inc/)

Ensure our repo files exist in Inc/ and Src/
For include files, nothing needs to be done, but for source files, you need to add them to IAR:
In IAR, right-click the (project)/Application/User group and click Add. Select all of our .c files and add them.

/* old changes */
main.c additions { // place into appropriate positions in file
#include "quadcontrol.h"
#include "bno055.h"
#include "accel.h"

BNO055_Init_I2C(&hi2c1); // right after all the MX_I2C1_Init() init function calls

uint8_t imu_readings[IMU_NUMBER_OF_BYTES];
int16_t accel_data[3];
float acc_x, acc_y, acc_z;

GetAccelData(&hi2c1, (uint8_t*)imu_readings);
accel_data[0] = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);      // Turn the MSB and LSB into a signed 16-bit value
accel_data[1] = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
accel_data[2] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
acc_x = ((float)(accel_data[0]))/100.0f; //m/s2
acc_y = ((float)(accel_data[1]))/100.0f;
acc_z = ((float)(accel_data[2]))/100.0f;
int len = sprintf((char*) strBuf, "X: %.2f Y: %.2f Z: %.2f\r\n", acc_x, acc_y, acc_z);
CDC_Transmit_FS(strBuf, len);
/////////////////

/* Repo version */
Check out all files, and make the following additions to main.c:
Add #include "quadcontrol.h" in the user includes section
Replace while(1) loop in main() with a call to quadcontrol().

// All done, click the compile and run button, then connect a microUSB to the VCOM port of the STM32 and use Arduino serial monitor to view output.
