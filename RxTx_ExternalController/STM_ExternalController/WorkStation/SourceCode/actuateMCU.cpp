
#include "./../IncludeFiles/actuateMCU.h"

int main()
{

  // Declared Variables
  int clkDelay = 10 * 100000; // 1,000,000
  char test0Array[] = { "This is a test run.  Test 0\r\n" };
  char test1Array[] = { "This is a test run.  Test 1\r\n" };

  /*       GENERATED_CODE_HERE      */
  STM_Config();

  /*     CONFIGURATION_CODE_HERE    */
  //USART6_Config();
  txBufferUSART6(sizeof(test0Array), test0Array);
  txBufferUSART6(sizeof(test1Array), test1Array);

  // Main loop
  while(1)
  {
    turnOnGPIO_LEDs(clkDelay);
    turnOffGPIO_LEDs(clkDelay);
  }

  return 0;
}
