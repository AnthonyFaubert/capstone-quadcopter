

#include "CC1101.h"

CC1101_MEM CC1101;
CC1101_TRANSMIT_FRAME transmitFrame;
CC1101_RECEIVE_FRAME receiveFrame;

volatile uint8_t CS_Recieved;
uint8_t receivedMessages;

enum {commandStr, readOrWrite, burstW, burstR, reset} UserState;

void CC1101_Read(uint8_t readAddress, uint8_t checkCS, uint8_t statusReg)
{
  if(readAddress < 0x30 || (statusReg && readAddress >= 0x30) )
  {
    if(UserState != readOrWrite)
    {
      SPI_DMA_Config(RECEIVE_FRAME_SIZE,
                     (uint32_t) &receiveFrame,
                     TRANSMIT_FRAME_SIZE,
                     (uint32_t) &transmitFrame
                    );
      UserState = readOrWrite;
    }
    transmitFrame.header = R_N_W | readAddress;
    if(statusReg)
    {
      transmitFrame.header |= BURST;
    }
    transmitFrame.data = 0x00;
    SPI_TX();
    if(checkCS)
    {
      checkChipStatus();
    }
  }
}

void CC1101_BurstRead()
{

}

void CC1101_Write(uint8_t writeAddress, uint8_t data, uint8_t checkCS)
{
  if(writeAddress < 0x30)
  {
    if(UserState != readOrWrite)
    {
      SPI_DMA_Config(RECEIVE_FRAME_SIZE,
                     (uint32_t) &receiveFrame,
                     TRANSMIT_FRAME_SIZE,
                     (uint32_t) &transmitFrame
                    );
      UserState = readOrWrite;
    }
    transmitFrame.header = writeAddress;
    transmitFrame.data = data;
    SPI_TX();
    if(checkCS)
    {
      checkChipStatus();
    }
  }
}

void CC1101_BurstWrite()
{

}

void CC1101_CommandStrobe(uint8_t command, uint8_t checkCS)
{
  uint8_t commandAddr = command & 0x3F;
  if((commandAddr >= 0x30) && (commandAddr <= 0x3D))
  {
    if(UserState != commandStr)
    {
      SPI_DMA_Config(COMMAND_STROBE_SIZE,
                     (uint32_t) &receiveFrame,
                     COMMAND_STROBE_SIZE,
                     (uint32_t) &transmitFrame
                    );
      UserState = commandStr;
    }
    transmitFrame.header = command;
    transmitFrame.data = 0x00;
    short val = eightBitToHex(transmitFrame.header);
    txBufferUSART6(sizeof(short), (char *) &val);
    SPI_TX();
    if(checkCS)
    {
      checkChipStatus();
    }
  }
}

char genericMessage[] = { "Received SPI Message #__:" };
char byte0Message[] = { "CHIP_STATUS: 0x__; " };
char byte1Message[] = { "DATA_OUT: 0x__" };

void checkChipStatus()
{
  while(!CS_Recieved);
  CS_Recieved = 0;
  receivedMessages++;
  uint16_t messageNumber = eightBitToHex(receivedMessages);
  uint16_t chipStatus = eightBitToHex(receiveFrame.chipStatus);
  uint16_t data = eightBitToHex(receiveFrame.data);
  *(genericMessage + 22) = (char) ((messageNumber >> 8) & 0xFF);
  *(genericMessage + 23) = (char) (messageNumber & 0xFF);
  txBufferUSART6(sizeof(genericMessage), genericMessage);
  newLine();
  *(byte0Message + 15) = (char) ((chipStatus >> 8) & 0xFF);
  *(byte0Message + 16) = (char) (chipStatus & 0xFF);
  txBufferUSART6(sizeof(byte0Message), byte0Message);
  *(byte1Message + 12) = (char) ((data >> 8) & 0xFF);
  *(byte1Message + 13) = (char) (data & 0xFF);
  txBufferUSART6(sizeof(byte1Message), byte1Message);
  newLine();
}

void CC1101_Configure(
                      uint32_t propagationAddress,
                      uint32_t PA_Size,
                      uint32_t targetAddress,
                      uint32_t TA_Size
                     )
{

}

int8_t CC1101_Reset()
{
  // On/off for cold start
  GPIOA->ODR |= GPIO_ODR_OD4;
  for(int i = 0; i < 100000; i++);
  GPIOA->ODR &= ~GPIO_ODR_OD4;
  for(int i = 0; i < 100000; i++);
  GPIOA->ODR |= GPIO_ODR_OD4;
  for(int i = 0; i < 100000; i++);
  CC1101_CommandStrobe(SRES, 1); // SystemReset
  for(int i = 0; i < 100000; i++);
  //while(GPIOA->IDR & GPIO_IDR_ID6);
  while(receiveFrame.data != CC1101_VERSION)
  {
    CC1101_Read(0x31, 1, 1);
  }
  return CC1101_RESET_SUCCESS;
}

char welcomeMessage[] = { "Welcome to the CC1101 Chip-Status Message Navigator" };
char resetMessage[] = {"Reseting CC1101 Now"};
char successMessage[] = { "CC1101 succesfully reset" };
char failureMessage[] = { "CC1101 unsuccessfully reset" };

void CC1101_Test()
{
  SPI_Init();
  UserState = reset;
  CS_Recieved = 0;
  receivedMessages = 0;
  receiveFrame.chipStatus = 0x00;
  receiveFrame.data = 0x00;
  txBufferUSART6(sizeof(welcomeMessage), welcomeMessage);
  newLine();
  txBufferUSART6(sizeof(resetMessage), resetMessage);
  newLine();
  newLine();
  CC1101_Reset();
  /*if()
  {
    txBufferUSART6(sizeof(successMessage), successMessage);
    newLine();
    CC1101_Read(0x31, 1, 1);
    CC1101_Read(0x31, 1, 1);
    CC1101_Read(0x31, 1, 1);
  }
  else
  {
    txBufferUSART6(sizeof(failureMessage), failureMessage);
  }*/
  CC1101_Read(0x31, 1, 1);
  CC1101_Read(0x31, 1, 1);
  CC1101_Read(0x31, 1, 1);
  for(;;){}
  for(int i = 0; i < 10; i++)
  {
    CC1101_CommandStrobe(SNOP, 1);
    CC1101_Read(0x00, 1, 0);
  }
  while(1);
  CC1101_CommandStrobe(SNOP, 1); // Message1
  CC1101_CommandStrobe(SNOP, 1); // Message1
  CC1101_CommandStrobe(SNOP, 1); // Message1
  while(1);
/*  CC1101_CommandStrobe(SNOP, 1); // Message1
  CC1101_CommandStrobe(SNOP, 1); // Message1
  CC1101_Read(0x00, 1);
  CC1101_Read(0x00, 1);
  CC1101_Read(0x00, 1);
  CC1101_Read(0x01, 1);
  CC1101_Read(0x02, 1);
  CC1101_Read(0x03, 1);
  CC1101_Read(0x04, 1);
  CC1101_Read(0x05, 1);
  while(1); */
}
