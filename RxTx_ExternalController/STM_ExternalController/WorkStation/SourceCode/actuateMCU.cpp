
#include "./../IncludeFiles/actuateMCU.h"

int main()
{

  // Declared Variables

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

  /*     FUNCTION_CALLS_HERE     */
// char testArray0[] = { "Test Code 1 fa3d\r\n" };
// char testArray1[] = { "Test Code 2 ddsa\r\n" };
// txBufferUSART3(sizeof(testArray0), testArray0);
// txBufferUSART3(sizeof(testArray1), testArray1);
USART3_ReturnToSender();
LED_Display();
  //CC1101_Test();
  //LED_Display();

  // Main loop
  while(1)
  {}

  return 0;
}
