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
#ifndef __LSM9DS0_H__
#define __LSM9DS0_H__

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>

#include "i2c_master.h"
#include "vector.h"

#define LSM9DS0_ADDRESS_ACCELMAG (0x1D << 1)
#define LSM9DS0_ADDRESS_GYRO     (0x6B << 1)
#define LSM9DS0_XM_ID            (0b01001001)
#define LSM9DS0_G_ID             (0b11010100)

typedef enum
{
    LSM9DS0_ACCELRANGE_2G           = (0b000 << 3),
    LSM9DS0_ACCELRANGE_4G           = (0b001 << 3),
    LSM9DS0_ACCELRANGE_6G           = (0b010 << 3),
    LSM9DS0_ACCELRANGE_8G           = (0b011 << 3),
    LSM9DS0_ACCELRANGE_16G          = (0b100 << 3)
} lsm9ds0AccelRange_t;

typedef enum
{
    LSM9DS0_MAGGAIN_2GAUSS          = (0b00 << 5),  // +/- 2 gauss
    LSM9DS0_MAGGAIN_4GAUSS          = (0b01 << 5),  // +/- 4 gauss
    LSM9DS0_MAGGAIN_8GAUSS          = (0b10 << 5),  // +/- 8 gauss
    LSM9DS0_MAGGAIN_12GAUSS         = (0b11 << 5)   // +/- 12 gauss
} lsm9ds0MagGain_t;

typedef enum
{
    LSM9DS0_GYROSCALE_245DPS        = (0b00 << 4),  // +/- 245 degrees per second rotation
    LSM9DS0_GYROSCALE_500DPS        = (0b01 << 4),  // +/- 500 degrees per second rotation
    LSM9DS0_GYROSCALE_2000DPS       = (0b10 << 4)   // +/- 2000 degrees per second rotation
} lsm9ds0GyroScale_t;

/*
 * Functia initializeaza senzorii (accelerometru, gyroscop, magentometru)
 *
 * @return - true on success, false otherwise
 */
bool LSM9DS0_init(void);

/*
 * Functia citeste si salveaza in 'accelData' valorile acceleratiei curente
 * de pe fiecare axa X, Y, Z.
 *
 * Unitatea de masura este: mG.
 *   1 G ~= 9.8m/2^2
 *   1 G = 1000 mG
 */
void LSM9DS0_readAccel(vector3f_t *accelData);

/*
 * Functia citeste si salveaza in 'magData' valorile campului magnetic curente
 * de pe fiecare axa X, Y, Z.
 *
 * Unitatea de masura este: mGauss.
 *   1 Gauss = 1000 mGauss
 */
void LSM9DS0_readMag(vector3f_t *magData);

/*
 * Functia citeste si salveaza in 'gyroData' valorile vitezei unghiulare
 * de pe fiecare axa X, Y, Z.
 *
 * Unitatea de masura este: DPS - Degrees-Per-Second.
 */
void LSM9DS0_readGyro(vector3f_t *gyroData);

/*
 * Functia configureaza rezolutia accelerometrului.
 */
void LSM9DS0_setupAccel(lsm9ds0AccelRange_t range);

/*
 * Functia configureaza rezolutia magnetometrului.
 */
void LSM9DS0_setupMag(lsm9ds0MagGain_t gain);

/*
 * Functia configureaza rezolutia giroscopului.
 */
void LSM9DS0_setupGyro(lsm9ds0GyroScale_t scale);

#endif

