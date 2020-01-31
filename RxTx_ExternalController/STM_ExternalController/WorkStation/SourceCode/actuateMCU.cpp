
#include "./../IncludeFiles/actuateMCU.h"

int rxReceived;
int LED_ON;

int main()
{

  // Declared Variables
  int clkDelay = 100 * 100000; // 1,000,000
  char test0Array[] = { "This is a test run.  Test 0\r\n" };
  char test1Array[] = { "This is a test run.  Test 1\r\n" };

  uint32_t bufferSize = 8;
  char receiveAddress[8];

  char sendValue = '!';

  /*  CONTROLLER_DYNAMICS_INFO  */
  // BIG_ENDIAN //

  // LEFT-RIGHT //
  // MSB (Byte 1)
  // Byte 2

  // UP-DOWN //
  // Byte 3
  // Byte 4

  // THRUST //
  // Byte 5
  // Byte 6

  // THRUST //
  // Byte 7
  // Byte 8

  // UNUSED_SHORT //
  // Byte 9
  // LSB (Byte 10)

  /*       GENERATED_CODE_HERE      */
  STM_Config();

  /*     CONFIGURATION_CODE_HERE    */
  //USART6->CR1 |= USART_CR1_RXNEIE; // Disable USART6_Receive_Interrupt
  LED_ON = 0;
  //USART6->CR1 &= ~USART_CR1_TCIE;
  USART6_RX_Config(bufferSize, receiveAddress);

  /*  Tx_Functions  */
  // txBufferUSART6(sizeof(test0Array), test0Array);
  // txBufferUSART6(sizeof(test1Array), test1Array);

  // Main loop
  while(1)
  {

    if(rxReceived)
    {
      txBufferUSART6(bufferSize, receiveAddress);
    }
    // LED Display
  //  turnOnGPIO_LEDs(clkDelay);
  //  turnOffGPIO_LEDs(clkDelay);
  }

  return 0;
}
