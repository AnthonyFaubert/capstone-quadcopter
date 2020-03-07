
#include "IMU.h"
#include "i2c.h"
#include "bno055.h"

// Gyro config
static uint8_t GPwrMode = NormalG;     // Gyro power mode
static uint8_t Gscale   = GFS_2000DPS; // Gyro full scale
//uint8_t Godr	 	= GODR_250Hz;  // Gyro sample rate
static uint8_t Gbw      = GBW_116Hz;   // Gyro bandwidth
// Accel config
static uint8_t Ascale   = AFS_16G;   // Accel full scale
static uint8_t APwrMode = NormalA;   // Accel power mode
static uint8_t Abw      = ABW_250Hz; // Accel bandwidth, accel sample rate divided by ABW_divx
// Mag config
//uint8_t Mscale 	= MFS_4Gauss;      // Select magnetometer full-scale resolution
static uint8_t MOpMode  = EnhancedRegular; // Select magnetometer perfomance mode
static uint8_t MPwrMode = Normal;          // Select magnetometer power mode
static uint8_t Modr     = MODR_30Hz;       // Select magnetometer ODR when in BNO055 bypass mode

// Select BNO055 power mode
static uint8_t PWRMode = Normalpwr;
// specify operation mode for sensors [ACCONLY|MAGONLY|GYROONLY|ACCMAG|ACCGYRO|MAGGYRO|AMG|NDOF|NDOF_FMC_OFF]
static uint8_t OPRMode = NDOF;

// TODO: remove this commented out code
/*
// BNO055 data status register
static uint8_t status;
// scale resolutions per LSB for the sensors
static float aRes, gRes, mRes;

// IMU calibration variables
static uint8_t cal_sys  = 0;
static uint8_t cal_gyro = 0;
static uint8_t cal_acc  = 0;
static uint8_t cal_mag  = 0;
static uint8_t cal_imu  = 0;

// Read number of bytes from IMU (24 for ACCGYRO; 38 for NDOF)
static const uint8_t num_of_bytes_read = 18; // FIXME: oh shit, how has this been working?

static const char read_devid[] = {START_BYTE, REG_READ, BNO055_CHIP_ID, 0x01};
//const char read_acc[]        = {REG_READ, BNO055_ACC_DATA_X_LSB, num_of_bytes_read};
static const char read_calib[2] = {REG_READ, BNO055_CALIB_STAT};
static const char reset_sensor[3] = {REG_WRITE, BNO055_SYS_TRIGGER, 0x01 << 5};
static uint8_t get_readings[1] = {BNO055_ACC_DATA_X_LSB};
*/

extern I2C_HandleTypeDef hi2c1;
// Configure BNO sensor
void IMUInit() {
  I2C_HandleTypeDef* hi2c_device = &hi2c1;
  // Select BNO055 config mode
  uint8_t opr_config_mode[2] = {BNO055_OPR_MODE, CONFIGMODE};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, opr_config_mode, sizeof(opr_config_mode), 10);
  HAL_Delay(10);

  // Select page 1 to configure sensors
  uint8_t conf_page1[2] = {BNO055_PAGE_ID, 0x01};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_page1, sizeof(conf_page1), 10);
  HAL_Delay(10);

  // Configure ACC (Page 1; 0x08)
  uint8_t conf_acc[2] = {BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_acc, sizeof(conf_acc), 10);
  HAL_Delay(10);

  // Configure GYR
  uint8_t conf_gyro[2] = {BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_gyro, sizeof(conf_gyro), 10);
  HAL_Delay(10);

  uint8_t conf_gyro_pwr[2] = {BNO055_GYRO_CONFIG_1, GPwrMode};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_gyro_pwr, sizeof(conf_gyro_pwr), 10);
  HAL_Delay(10);

  // Configure MAG
  uint8_t conf_mag_pwr[4] = {REG_WRITE, BNO055_MAG_CONFIG, 0x01, MPwrMode << 5 | MOpMode << 3 | Modr};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_mag_pwr, sizeof(conf_mag_pwr), 10);
  HAL_Delay(10);

  // Select BNO055 gyro temperature source 
  //PutHexString(START_BYTE, BNO055_TEMP_SOURCE, 0x01 );
	
  // Select page 0
  uint8_t conf_page0[2] = {BNO055_PAGE_ID, 0x00};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_page0, sizeof(conf_page0), 10);
  HAL_Delay(10);

  // Select BNO055 sensor units (Page 0; 0x3B, default value 0x80)
  /*- ORIENTATION_MODE		 - Android					(default)
    - VECTOR_ACCELEROMETER - m/s^2  					(default)
    - VECTOR_MAGNETOMETER  - uT								(default)
    - VECTOR_GYROSCOPE     - rad/s        v		(must be configured)
    - VECTOR_EULER         - degrees					(default)
    - VECTOR_LINEARACCEL   - m/s^2        v		(default)
    - VECTOR_GRAVITY       - m/s^2						(default)
  */
  //const char conf_units[4] = {REG_WRITE, BNO055_UNIT_SEL, 0x01, 0x82};
  //SendAccelData(USART1, (uint8_t*)conf_units);
  //HAL_Delay(50);

  // Select BNO055 system power mode (Page 0; 0x3E)
  uint8_t pwr_pwrmode[2] = {BNO055_PWR_MODE, PWRMode};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, pwr_pwrmode, sizeof(pwr_pwrmode), 10);
  HAL_Delay(10);

  // Select BNO055 system operation mode (Page 0; 0x3D)
  uint8_t opr_oprmode[2] = {BNO055_OPR_MODE, OPRMode};
  HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, opr_oprmode, sizeof(opr_oprmode), 10);
  HAL_Delay(50);
}

// Send data to BNO055 over I2C
uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {} 
	return status;
}
	
// TBD
/*void readAccelData(int16_t *destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;      // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}*/

// TBD
uint8_t GetAccelChipId(I2C_HandleTypeDef* hi2c_device, uint8_t *chip_id) {
  return HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_CHIP_ID, I2C_MEMADD_SIZE_8BIT, chip_id, 1, 100);
}

// TBD
uint8_t GetAccelTemp(I2C_HandleTypeDef* hi2c_device) {
  uint8_t temp;
  HAL_I2C_Mem_Read_DMA(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_TEMP, I2C_MEMADD_SIZE_8BIT, &temp, 1);
  while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {} 
  return temp;
}

// Get IMU calibration values
uint8_t BNO055_Get_Calibration(I2C_HandleTypeDef* hi2c_device) {
  uint8_t calibration;
  HAL_I2C_Mem_Read_DMA(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_CALIB_STAT, I2C_MEMADD_SIZE_8BIT, &calibration, 1);
  while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY);
  return calibration;
}

// Calculate IMU calibration values
void BNO055_Calc_Calibration(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro, uint8_t *cal_acc, uint8_t *cal_mag) {
  *cal_system = (calibration >> 6) & 0x03;
  *cal_gyro   = (calibration >> 4) & 0x03;
  *cal_acc    = (calibration >> 2) & 0x03;
  *cal_mag    = (calibration) & 0x03;
}


// Get and store the current orientation quaternion into the pointer given, then return true if there was an error
bool IMUGetOrientation(Quaternion* quatDat) {
  const float scale = 1.0f / (1<<14);
  uint8_t readings[IMU_NUMBER_OF_BYTES];
  uint8_t status = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO<<1, BNO055_QUA_DATA_W_LSB, I2C_MEMADD_SIZE_8BIT, readings, IMU_NUMBER_OF_BYTES, 100);
  
  int16_t w = (int16_t)(readings[1] << 8) | (int16_t)(readings[0]);
  int16_t x = (int16_t)(readings[3] << 8) | (int16_t)(readings[2]);
  int16_t y = (int16_t)(readings[5] << 8) | (int16_t)(readings[4]);
  int16_t z = (int16_t)(readings[7] << 8) | (int16_t)(readings[6]);
  quatDat->w = scale * ((float) w);
  quatDat->x = scale * ((float) x);
  quatDat->y = scale * ((float) y);
  quatDat->z = scale * ((float) z);
  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
  return status == HAL_OK; // FIXME: is this right?
}

// Get and store the current gyro data into the pointer given, then return true if there was an error
bool IMUGetGyro(GyroData* gyroData) {
  const float scale = 1.0f / 16.0f;
  uint8_t readings[IMU_NUMBER_OF_BYTES];
  uint8_t status = HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO<<1, BNO055_GYR_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, readings, IMU_NUMBER_OF_BYTES, 100);

  int16_t x = (int16_t)(readings[1] << 8) | (int16_t)(readings[0]);
  int16_t y = (int16_t)(readings[3] << 8) | (int16_t)(readings[2]);
  int16_t z = (int16_t)(readings[5] << 8) | (int16_t)(readings[4]);
  gyroData->x = scale * ((float) x);
  gyroData->y = scale * ((float) y);
  gyroData->z = scale * ((float) z);
  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {} 
  return status == HAL_OK;// FIXME: is this right?
}
