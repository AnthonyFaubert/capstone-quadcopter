

#include "./../IncludeFiles/CC1101.h"

CC1101_MEM CC1101;
CC1101_DATA_FRAME dataFrame;

uint8_t chipStatus;
uint8_t commandByte;

enum {commandOrRead, write, burst, reset} UserState;

void CC1101_Read(uint8_t readAddress)
{
  if(readAddress < 0x30)
  {
    if(UserState != commandOrRead)
    {
      SPI_DMA_Config(sizeof(chipStatus), (uint32_t) &chipStatus,
                     sizeof(commandByte), (uint32_t) &commandByte
                    );
      UserState = commandOrRead;
    }
    commandByte = R_N_W | readAddress;
    SPI_TX();
  }
}

void CC1101_BurstRead()
{

}

void CC1101_Write(uint8_t writeAddress, uint8_t data)
{
  if(writeAddress < 0x30)
  {
    dataFrame.header = writeAddress;
    dataFrame.data = data;
    SPI_TX();
  }
}

void CC1101_BurstWrite()
{

}

void CC1101_CommandStrobe(uint8_t command)
{
  uint8_t commandAddr = command & 0x3F;
  if((commandAddr >= 0x30) && (commandAddr <= 0x3D))
  {
    if(UserState != commandOrRead)
    {
      SPI_DMA_Config(sizeof(chipStatus), (uint32_t) &chipStatus,
                     sizeof(commandByte),(uint32_t) &commandByte
                    );
      UserState = commandOrRead;
    }
    commandByte = command;
    SPI_TX();
  }
}

void CC1101_Configure(
                      uint32_t propagationAddress,
                      uint32_t PA_Size,
                      uint32_t targetAddress,
                      uint32_t TA_Size
                     )
{

}

void CC1101_Test()
{
  SPI_Init();
  UserState = reset;
  CC1101_CommandStrobe(SNOP);
}
