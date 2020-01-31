

#include "./../IncludeFiles/CC1101.h"


// Enable SPI1
// Enable DMAENTX (Later, DMAENRX)

void CC1101_Configure(
                      uint32_t propagationAddress, uint32_t PA_Size
                      uint32_t targetAddress, uint32_t TA_Size
                     )
{
  SPI_Init(PA_Size, propagationAddress, TA_Size, targetAddress);
}

void readSPI(short readAddress)
{

}

void burstReadSPI()
{

}

void writeSPI(short writeAddress, short data)
{

}

void burstWriteSPI()
{

}
