

#ifndef CC1101_H
#define CC1101_H

#ifdef __cplusplus
extern "C" {
#endif

/*  Include Files  */

// Non-Local Inc
#include "./../../CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

// Local Inc
#include "./USART.h"
#include "./GPIO.h"
#include "./SPI.h"
#include "./testLibrary.h"

/*  Declared Functions  */
void CC1101_Read(uint8_t readAddress, uint8_t checkCS, uint8_t statusReg);
void CC1101_BurstRead(void);

void CC1101_Write(uint8_t writeAddress, uint8_t data, uint8_t checkCS);
void CC1101_BurstWrite(void);

void CC1101_CommandStrobe(uint8_t command, uint8_t checkCS);

void checkChipStatus(void);

void CC1101_Configure(
                      uint32_t propagationAddress,
                      uint32_t PA_Size,
                      uint32_t targetAddress,
                      uint32_t TA_Size
                     );
void CC1101_Test(void);

/*   Defined Values/Macros   */
//////////////////// Frame Size (BYTES/2) ////////////////////
#define RECEIVE_FRAME_SIZE  (uint32_t) 0x2
#define TRANSMIT_FRAME_SIZE (uint32_t) 0x2
#define COMMAND_STROBE_SIZE (uint32_t) 0x1
//////////////////////////////////////////////////////////////

//////////////////// HEADER BYTE ////////////////////
#define R_N_W     (uint8_t) (0x1 << 7) // R_not_Write
#define BURST     (uint8_t) (0x1 << 6)
#define ADDR_MSK  (uint8_t) 0x3F
/////////////////////////////////////////////////////

//////////////////// CHIP-STATUS BYTE ////////////////////
#define CHIP_RDYn (uint8_t) (0x1 << 7)
#define STATE     (uint8_t) (0x7 << 4)
#define BYTES_AVL (uint8_t) 0xF
//////////////////////////////////////////////////////////

//////////////////// Command Strobes /////////////////////
#define BASE_COMMAND (uint8_t) (R_N_W | 0x30)
#define SRES         (uint8_t)  BASE_COMMAND        // System-Reset
#define SFSTXON      (uint8_t) (BASE_COMMAND + 0x1) // En & calibrate frequency syntehsizer
#define SXOFF        (uint8_t) (BASE_COMMAND + 0x2) // Turn off crystall oscillator
#define SCAL         (uint8_t) (BASE_COMMAND + 0x3) // Calibrate & turn off frequency syntehsizer
#define SRX          (uint8_t) (BASE_COMMAND + 0x4) // Enable RX
#define STX          (uint8_t) (BASE_COMMAND + 0x5) // Enable TX
#define SIDLE        (uint8_t) (BASE_COMMAND + 0x6) // Enable Idle
#define SWORE        (uint8_t) (BASE_COMMAND + 0x8) // Start automatic RX polling sequence
#define SPWD         (uint8_t) (BASE_COMMAND + 0x9) // Enter Power Down when CSn -> High
#define SFRX         (uint8_t) (BASE_COMMAND + 0xA) // FLUSH Rx FIFO
#define SFTX         (uint8_t) (BASE_COMMAND + 0xB) // FLUSH Tx FIFO
#define SWORRST      (uint8_t) (BASE_COMMAND + 0xC) // Reset real-time clock to Event 1
#define SNOP         (uint8_t) (BASE_COMMAND + 0xD) // NO-OP
//////////////////////////////////////////////////////////

/*  Declared Variables  */

extern volatile uint8_t CS_Recieved;

// Basic Data-Frame Format
typedef struct
{
  uint8_t data;
  uint8_t header;
} CC1101_TRANSMIT_FRAME;

// Basic Receive-Frame Format
typedef struct
{
  uint8_t data;
  uint8_t chipStatus;
} CC1101_RECEIVE_FRAME;

// In/Out Pin Configuration
typedef struct
{
  uint8_t IOCFG2;
  uint8_t IOCFG1;
  uint8_t IOCFG0;
} IOCFG_TypeDef;

// Sync Word
typedef struct
{
  uint8_t SYNC1;
  uint8_t SYNC0;
} SYNC_TypeDef;

// Packet Configuration
typedef struct
{
  uint8_t PKTLEN;
  uint8_t PKTCTRL1;
  uint8_t PKTCTRL0;
} PKT_TypeDef;

// Frequency Synthesizer Control
typedef struct
{
  uint8_t FSCTRL1;
  uint8_t FSCTRL0;
} FSCTRL_TypeDef;

// Frequency Control Word
typedef struct
{
  uint8_t FREQ2;
  uint8_t FREQ1;
  uint8_t FREQ0;
} FREQ_TypeDef;

// Modem Configuration
typedef struct
{
  uint8_t MDMCFG4;
  uint8_t MDMCFG3;
  uint8_t MDMCFG2;
  uint8_t MDMCFG1;
  uint8_t MDMCFG0;
} MDMCFG_TypeDef;

// Main Radio Control State Machine
typedef struct
{
  uint8_t MCSM2;
  uint8_t MCSM1;
  uint8_t MCSM0;
} MCSM_TypeDef;

// AGC Control
typedef struct
{
  uint8_t AGCTRL2;
  uint8_t AGCTRL1;
  uint8_t AGCTRL0;
} AGCTRL_TypeDef;

// Event 0 Timeout (Wake-on-Radio event)
typedef struct
{
  uint8_t WOREVT1;
  uint8_t WOREVT0;
  uint8_t WORCTRL;
} WOREVT_TypeDef;

// Front-End Configuration
typedef struct
{
  uint8_t FREND1;
  uint8_t FREND0;
} FREND_TypeDef;

// Frequency Synthesizer Calibration
typedef struct
{
  uint8_t FSCAL3;
  uint8_t FSCAL2;
  uint8_t FSCAL1;
  uint8_t FSCAL0;
} FSCAL_TypeDef;

// RC Oscillator Configuration
typedef struct
{
  uint8_t RCCTRL1;
  uint8_t RCCTRL0;
} RCCTRL_TypeDef;

// Various test settings
typedef struct
{
  uint8_t TEST2;
  uint8_t TEST1;
  uint8_t TEST0;
} TEST_TypeDef;

// Wake-on-Radio Timer
typedef struct
{
  uint8_t WORTIME1;
  uint8_t WORTIME0;
} WORTIME_TypeDef;

// Last RC oscillator calibration result
typedef struct
{
  uint8_t RCCTRL1_STATUS;
  uint8_t RCCTRL0_STATUS;
} RCCTRL_STATUS_TypeDef;

// Memory Layout:  CC1101 Chip
typedef struct
{
    IOCFG_TypeDef IOCFG;                                             // [0x00 - 0x02]
    uint8_t FIFOTHR; // Rx FIFO and Tx FIFO thresholds                  [0x03]
    SYNC_TypeDef SYNC;                                               // [0x04-0x05]
    PKT_TypeDef PKT;                                                 // [0x06-0x08]
    uint8_t ADDR; // Device Address                                     [0x09]
    uint8_t CHANNR; // Channel Number                                   [0x0A]
    FSCTRL_TypeDef FSCTRL;                                           // [0x0B-0x0C]
    FREQ_TypeDef FREQ;                                               // [0x0D-0x0F]
    MDMCFG_TypeDef MDMCFG;                                           // [0x10-0x14]
    uint8_t DEVIATN; // Modem Deviation Setting                         [0x15]
    MCSM_TypeDef MCSM;                                               // [0x16-0x18]
    uint8_t FOCCFG; // Frequency Offset Compensation Configuration      [0x19]
    uint8_t BSCFG; // Bit Synchronization Configuration                 [0x1A]
    AGCTRL_TypeDef AGCTRL;                                           // [0x1B-0x1D]
    WOREVT_TypeDef WORVET;                                           // [0x1E-0x20]
    FREND_TypeDef FREND;                                             // [0x21-0x22]
    FSCAL_TypeDef FSCAL;                                             // [0x23-0x26]
    RCCTRL_TypeDef RCCTRL;                                           // [0x27-0x28]
    uint8_t FSTEST; // Frequency Synthesizer Calibration Control        [0x29]
    uint8_t PTEST; // Production Test                                   [0x2A]
    uint8_t AGCTEST; // AGC Test                                        [0x2B]
    TEST_TypeDef TEST;                                               // [0x2C-0x2E]
    uint8_t RESERVED;                                                // [0x2F]
    uint8_t PARTNUM; // Part Number                                     [0x30]
    uint8_t VERSION; // Version Number                                  [0x31]
    uint8_t FREQEST; // Frequency Offest Estimate                       [0x32]
    uint8_t LQI; // Demodular Esimate for Link Quality                  [0x33]
    uint8_t RSSI; // Recieved Signal Strength Indication                [0x34]
    uint8_t MARCSTATE; // Control State Machine State                   [0x35]
    WORTIME_TypeDef WORTIME;                                         // [0x36-0x37]
    uint8_t PKTSTATUS; // Current GDOx status and packet status         [0x38]
    uint8_t VCO_VC_DAC; // Current setting from PLL calibration module  [0x39]
    uint8_t TXBYTES; // Underflow and number of bytes in TX FIFO        [0x3A]
    uint8_t RXBYTES; // Overflow and number of bytes in RX FIFO         [0x3B]
    RCCTRL_STATUS_TypeDef RCCTRL_STATUS;                             // [0x3C-0x3D]
} CC1101_MEM;

#ifdef __cplusplus
}
#endif

#endif // CC1101_H
