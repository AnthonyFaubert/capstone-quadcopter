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
// may have to set target microcontroller

