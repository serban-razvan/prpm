/***************************************************************************
  This is a library for the LSM9DS0 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM9DS0 Breakouts

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include "LSM9DS0.h"

/***************************************************************************
 PREPROCESSOR DEFINITION
 ***************************************************************************/

// Linear Acceleration: mg per LSB
#define LSM9DS0_ACCEL_MG_LSB_2G         (0.061F)
#define LSM9DS0_ACCEL_MG_LSB_4G         (0.122F)
#define LSM9DS0_ACCEL_MG_LSB_6G         (0.183F)
#define LSM9DS0_ACCEL_MG_LSB_8G         (0.244F)
#define LSM9DS0_ACCEL_MG_LSB_16G        (0.732F) // Is this right? Was expecting 0.488F

// Magnetic Field Strength: gauss range
#define LSM9DS0_MAG_MGAUSS_2GAUSS       (0.08F)
#define LSM9DS0_MAG_MGAUSS_4GAUSS       (0.16F)
#define LSM9DS0_MAG_MGAUSS_8GAUSS       (0.32F)
#define LSM9DS0_MAG_MGAUSS_12GAUSS      (0.48F)

// Angular Rate: dps per LSB
#define LSM9DS0_GYRO_DPS_DIGIT_245DPS   (0.00875F)
#define LSM9DS0_GYRO_DPS_DIGIT_500DPS   (0.01750F)
#define LSM9DS0_GYRO_DPS_DIGIT_2000DPS  (0.07000F)

// Temperature: LSB per degree celsius
#define LSM9DS0_TEMP_LSB_DEGREE_CELSIUS (8)  // 1°C = 8, 25° = 200, etc.

#define GYROTYPE                        (true)
#define XMTYPE                          (false)

/***************************************************************************
 PRIVATE TYPES
 ***************************************************************************/

typedef enum
{
    LSM9DS0_REGISTER_WHO_AM_I_G     = 0x0F,
    LSM9DS0_REGISTER_CTRL_REG1_G    = 0x20,
    LSM9DS0_REGISTER_CTRL_REG3_G    = 0x22,
    LSM9DS0_REGISTER_CTRL_REG4_G    = 0x23,
    LSM9DS0_REGISTER_OUT_X_L_G      = 0x28,
    LSM9DS0_REGISTER_OUT_X_H_G      = 0x29,
    LSM9DS0_REGISTER_OUT_Y_L_G      = 0x2A,
    LSM9DS0_REGISTER_OUT_Y_H_G      = 0x2B,
    LSM9DS0_REGISTER_OUT_Z_L_G      = 0x2C,
    LSM9DS0_REGISTER_OUT_Z_H_G      = 0x2D,
} lsm9ds0GyroRegisters_t;

typedef enum
{
    LSM9DS0_REGISTER_TEMP_OUT_L_XM  = 0x05,
    LSM9DS0_REGISTER_TEMP_OUT_H_XM  = 0x06,
    LSM9DS0_REGISTER_STATUS_REG_M   = 0x07,
    LSM9DS0_REGISTER_OUT_X_L_M      = 0x08,
    LSM9DS0_REGISTER_OUT_X_H_M      = 0x09,
    LSM9DS0_REGISTER_OUT_Y_L_M      = 0x0A,
    LSM9DS0_REGISTER_OUT_Y_H_M      = 0x0B,
    LSM9DS0_REGISTER_OUT_Z_L_M      = 0x0C,
    LSM9DS0_REGISTER_OUT_Z_H_M      = 0x0D,
    LSM9DS0_REGISTER_WHO_AM_I_XM    = 0x0F,
    LSM9DS0_REGISTER_INT_CTRL_REG_M = 0x12,
    LSM9DS0_REGISTER_INT_SRC_REG_M  = 0x13,
    LSM9DS0_REGISTER_CTRL_REG1_XM   = 0x20,
    LSM9DS0_REGISTER_CTRL_REG2_XM   = 0x21,
    LSM9DS0_REGISTER_CTRL_REG5_XM   = 0x24,
    LSM9DS0_REGISTER_CTRL_REG6_XM   = 0x25,
    LSM9DS0_REGISTER_CTRL_REG7_XM   = 0x26,
    LSM9DS0_REGISTER_OUT_X_L_A      = 0x28,
    LSM9DS0_REGISTER_OUT_X_H_A      = 0x29,
    LSM9DS0_REGISTER_OUT_Y_L_A      = 0x2A,
    LSM9DS0_REGISTER_OUT_Y_H_A      = 0x2B,
    LSM9DS0_REGISTER_OUT_Z_L_A      = 0x2C,
    LSM9DS0_REGISTER_OUT_Z_H_A      = 0x2D,
} lsm9ds0MagAccelRegisters_t;

typedef enum
{
    LSM9DS0_ACCELDATARATE_POWERDOWN = (0b0000 << 4),
    LSM9DS0_ACCELDATARATE_3_125HZ   = (0b0001 << 4),
    LSM9DS0_ACCELDATARATE_6_25HZ    = (0b0010 << 4),
    LSM9DS0_ACCELDATARATE_12_5HZ    = (0b0011 << 4),
    LSM9DS0_ACCELDATARATE_25HZ      = (0b0100 << 4),
    LSM9DS0_ACCELDATARATE_50HZ      = (0b0101 << 4),
    LSM9DS0_ACCELDATARATE_100HZ     = (0b0110 << 4),
    LSM9DS0_ACCELDATARATE_200HZ     = (0b0111 << 4),
    LSM9DS0_ACCELDATARATE_400HZ     = (0b1000 << 4),
    LSM9DS0_ACCELDATARATE_800HZ     = (0b1001 << 4),
    LSM9DS0_ACCELDATARATE_1600HZ    = (0b1010 << 4)
} lm9ds0AccelDataRate_t;

typedef enum
{
    LSM9DS0_MAGDATARATE_3_125HZ     = (0b000 << 2),
    LSM9DS0_MAGDATARATE_6_25HZ      = (0b001 << 2),
    LSM9DS0_MAGDATARATE_12_5HZ      = (0b010 << 2),
    LSM9DS0_MAGDATARATE_25HZ        = (0b011 << 2),
    LSM9DS0_MAGDATARATE_50HZ        = (0b100 << 2),
    LSM9DS0_MAGDATARATE_100HZ       = (0b101 << 2)
} lsm9ds0MagDataRate_t;

/***************************************************************************
 PRIVATE DATA
 ***************************************************************************/

static float cfg_accel_mg_lsb;
static float cfg_mag_mgauss_lsb;
static float cfg_gyro_dps_digit;

/***************************************************************************
 PRIVATE FUNCTIONS PROTOTYPES
 ***************************************************************************/

static void write8(bool type, uint8_t reg, uint8_t value);
static uint8_t read8(bool type, uint8_t reg);
static void readBuffer(bool type, uint8_t reg, uint8_t len, uint8_t *buffer);

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

bool LSM9DS0_init()
{
    uint8_t id = read8(XMTYPE, LSM9DS0_REGISTER_WHO_AM_I_XM);
    if (id != LSM9DS0_XM_ID)
       return false;

    id = read8(GYROTYPE, LSM9DS0_REGISTER_WHO_AM_I_G);
    if (id != LSM9DS0_G_ID)
       return false;

    // Enable the accelerometer continous
    write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG1_XM, 0x67); // 100hz XYZ
    write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, 0b11110000);
    // enable mag continuous
    write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG7_XM, 0b00000000);
    // enable gyro continuous
    write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG1_G, 0x0F); // on XYZ
    // enable the temperature sensor (output rate same as the mag sensor)
    uint8_t tempReg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM);
    write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, tempReg | (1<<7));

    // Set default ranges for the various sensors
    LSM9DS0_setupAccel(LSM9DS0_ACCELRANGE_2G);
    LSM9DS0_setupMag(LSM9DS0_MAGGAIN_2GAUSS);
    LSM9DS0_setupGyro(LSM9DS0_GYROSCALE_245DPS);

    return true;
}

void LSM9DS0_readAccel(vector3f_t *accelData)
{
  // Read the accelerometer
  uint8_t buffer[6];
  readBuffer(XMTYPE, 0x80 | LSM9DS0_REGISTER_OUT_X_L_A, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low uint8_t first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

  accelData->x = xhi * cfg_accel_mg_lsb;
  accelData->y = yhi * cfg_accel_mg_lsb;
  accelData->z = zhi * cfg_accel_mg_lsb;
}

void LSM9DS0_readMag(vector3f_t *magData)
{
  // Read the magnetometer
  uint8_t buffer[6];
  readBuffer(XMTYPE, 0x80 | LSM9DS0_REGISTER_OUT_X_L_M, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low uint8_t first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;
  magData->x = xhi * cfg_mag_mgauss_lsb;
  magData->y = yhi * cfg_mag_mgauss_lsb;
  magData->z = zhi * cfg_mag_mgauss_lsb;
}

void LSM9DS0_readGyro(vector3f_t *gyroData)
{
  // Read gyro
  uint8_t buffer[6];
  readBuffer(GYROTYPE, 0x80 | LSM9DS0_REGISTER_OUT_X_L_G, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low uint8_t first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

  gyroData->x = xhi * cfg_gyro_dps_digit;
  gyroData->y = yhi * cfg_gyro_dps_digit;
  gyroData->z = zhi * cfg_gyro_dps_digit;
}

int16_t LSM9DS0_readTemp()
{
  // Read temp sensor
  uint8_t buffer[2];
  readBuffer(XMTYPE, 0x80 | LSM9DS0_REGISTER_TEMP_OUT_L_XM, 2, buffer);
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];

  xhi <<= 8; xhi |= xlo;

  // Shift values to create properly formed integer (low uint8_t first)
  return xhi;
}

void LSM9DS0_setupAccel(lsm9ds0AccelRange_t range)
{
    uint8_t reg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG2_XM);
    reg &= ~(0b00111000);
    reg |= range;
    write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG2_XM, reg );

    switch (range)
    {
    case LSM9DS0_ACCELRANGE_2G:
        cfg_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_2G;
        break;

    case LSM9DS0_ACCELRANGE_4G:
        cfg_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_4G;
        break;

    case LSM9DS0_ACCELRANGE_6G:
        cfg_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_6G;
        break;

    case LSM9DS0_ACCELRANGE_8G:
        cfg_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_8G;
        break;

    case LSM9DS0_ACCELRANGE_16G:
        cfg_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_16G;
        break;
    }
}

void LSM9DS0_setupMag(lsm9ds0MagGain_t gain)
{
    uint8_t reg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG6_XM);
    reg &= ~(0b01100000);
    reg |= gain;
    write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG6_XM, reg );

    switch(gain)
    {
    case LSM9DS0_MAGGAIN_2GAUSS:
        cfg_mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_2GAUSS;
        break;

    case LSM9DS0_MAGGAIN_4GAUSS:
        cfg_mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_4GAUSS;
        break;

    case LSM9DS0_MAGGAIN_8GAUSS:
        cfg_mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_8GAUSS;
        break;

    case LSM9DS0_MAGGAIN_12GAUSS:
        cfg_mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_12GAUSS;
        break;
    }
}

void LSM9DS0_setupGyro(lsm9ds0GyroScale_t scale)
{
    uint8_t reg = read8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG4_G);
    reg &= ~(0b00110000);
    reg |= scale;
    write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG4_G, reg );

    switch(scale)
    {
    case LSM9DS0_GYROSCALE_245DPS:
        cfg_gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_245DPS;
        break;

    case LSM9DS0_GYROSCALE_500DPS:
        cfg_gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_500DPS;
        break;

    case LSM9DS0_GYROSCALE_2000DPS:
        cfg_gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_2000DPS;
        break;
    }
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

void write8(bool type, uint8_t reg, uint8_t value)
{
    uint8_t address;

    /* Who are we talking to? Gyro or Accel? */
    address = (type == GYROTYPE ? LSM9DS0_ADDRESS_GYRO :
               LSM9DS0_ADDRESS_ACCELMAG);

    (void)I2C_writeReg(address, reg, &value, 1);
}

uint8_t read8(bool type, uint8_t reg)
{
    uint8_t value = 0x00;
    uint8_t address;

    /* Who are we talking to? Gyro or Accel? */
    address = (type == GYROTYPE ? LSM9DS0_ADDRESS_GYRO :
               LSM9DS0_ADDRESS_ACCELMAG);

    (void)I2C_readReg(address, reg, &value, 1);

    return value;
}

void readBuffer(bool type, uint8_t reg, uint8_t len, uint8_t *buffer)
{
    uint8_t address;

    /* Who are we talking to? Gyro or Accel? */
    address = (type == GYROTYPE ? LSM9DS0_ADDRESS_GYRO :
               LSM9DS0_ADDRESS_ACCELMAG);

    (void)I2C_readReg(address, reg, buffer, len);
}

