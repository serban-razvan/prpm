/*!
 * @file Adafruit_MPL3115A2.cpp
 *
 * @mainpage Adafruit MPL3115A2 alitmeter
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's MPL3115A2 driver for the
 * Arduino platform.   It is designed specifically to work with the
 * Adafruit MPL3115A2 breakout: https://www.adafruit.com/products/1893
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include <avr/io.h>
#include <util/delay.h>

#include "I2C_master.h"
#include "MPL3115A2.h"
#include "usart.h"

/**************************************************************************/
/*!
    @brief  MPL3115A2 registers
*/
/**************************************************************************/

enum {
    MPL3115A2_REGISTER_STATUS        =       (0x00),

    MPL3115A2_REGISTER_PRESSURE_MSB     =    (0x01),
    MPL3115A2_REGISTER_PRESSURE_CSB    =     (0x02),
    MPL3115A2_REGISTER_PRESSURE_LSB    =     (0x03),

    MPL3115A2_REGISTER_TEMP_MSB       =      (0x04),
    MPL3115A2_REGISTER_TEMP_LSB      =       (0x05),

    MPL3115A2_REGISTER_DR_STATUS     =       (0x06),

    MPL3115A2_OUT_P_DELTA_MSB        =       (0x07),
    MPL3115A2_OUT_P_DELTA_CSB        =       (0x08),
    MPL3115A2_OUT_P_DELTA_LSB        =       (0x09),

    MPL3115A2_OUT_T_DELTA_MSB        =       (0x0A),
    MPL3115A2_OUT_T_DELTA_LSB        =       (0x0B),

    MPL3115A2_WHOAMI            =            (0x0C),

    MPL3115A2_BAR_IN_MSB         =           (0x14),
    MPL3115A2_BAR_IN_LSB         =           (0x15),
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 status register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_REGISTER_STATUS_TDR       = 0x02,
    MPL3115A2_REGISTER_STATUS_PDR       = 0x04,
    MPL3115A2_REGISTER_STATUS_PTDR      = 0x08,
};


/**************************************************************************/
/*!
    @brief  MPL3115A2 PT DATA register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_PT_DATA_CFG       = 0x13,
    MPL3115A2_PT_DATA_CFG_TDEFE     = 0x01,
    MPL3115A2_PT_DATA_CFG_PDEFE     = 0x02,
    MPL3115A2_PT_DATA_CFG_DREM      = 0x04,
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 control registers
*/
/**************************************************************************/
enum {

    MPL3115A2_CTRL_REG1     =    (0x26),
    MPL3115A2_CTRL_REG2         =   (0x27),
    MPL3115A2_CTRL_REG3         =   (0x28),
    MPL3115A2_CTRL_REG4         =   (0x29),
    MPL3115A2_CTRL_REG5         =    (0x2A),
};

/**************************************************************************/
/*!
    @brief  MPL3115A2 control register bits
*/
/**************************************************************************/
enum {
    MPL3115A2_CTRL_REG1_SBYB   = 0x01,
    MPL3115A2_CTRL_REG1_OST     = 0x02,
    MPL3115A2_CTRL_REG1_RST      = 0x04,
    MPL3115A2_CTRL_REG1_RAW     = 0x40,
    MPL3115A2_CTRL_REG1_ALT     = 0x80,
    MPL3115A2_CTRL_REG1_BAR     = 0x00,
};


/**************************************************************************/
/*!
    @brief  MPL3115A2 oversample values
*/
/**************************************************************************/
enum {
    MPL3115A2_CTRL_REG1_OS1     = 0x00,
    MPL3115A2_CTRL_REG1_OS2     = 0x08,
    MPL3115A2_CTRL_REG1_OS4     = 0x10,
    MPL3115A2_CTRL_REG1_OS8     = 0x18,
    MPL3115A2_CTRL_REG1_OS16        = 0x20,
    MPL3115A2_CTRL_REG1_OS32        = 0x28,
    MPL3115A2_CTRL_REG1_OS64        = 0x30,
    MPL3115A2_CTRL_REG1_OS128       = 0x38,
};

#define MPL3115A2_REGISTER_STARTCONVERSION (0x12) ///< start conversion

union ctrl_reg1_t {
    struct {
        uint8_t SBYB:1;
        uint8_t OST :1;
        uint8_t RST :1;
        uint8_t OS  :3;
        uint8_t RAW :1;
        uint8_t ALT :1;
    } bit;
    uint8_t reg;
} ctrl_reg1;

static void write8(uint8_t a, uint8_t d);
static uint8_t read8(uint8_t a);

/**************************************************************************/
/*!
      @brief   Setups the HW (reads coefficients values, etc.)
      @param twoWire Optional TwoWire I2C object
      @return true on successful startup, false otherwise
*/
/**************************************************************************/
bool MPL3115A2_init() {
    uint8_t whoami = read8(MPL3115A2_WHOAMI);

    if (whoami != MPL3115A2_WHOAMI_VAL) {
      return false;
    }

    write8(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
    _delay_ms(10);

    while(read8(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_RST)
       _delay_ms(1);

    ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS64 | MPL3115A2_CTRL_REG1_ALT;
    write8(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);

    write8(MPL3115A2_PT_DATA_CFG,
          MPL3115A2_PT_DATA_CFG_TDEFE |
          MPL3115A2_PT_DATA_CFG_PDEFE |
          MPL3115A2_PT_DATA_CFG_DREM);

    return true;
}

/**************************************************************************/
/*!
      @brief   Gets the doubleing-point pressure level in hPa
      @return pressure reading as a doubleing point value
*/
/**************************************************************************/
double MPL3115A2_getPressure() {
    uint8_t data[3] = {0};
    uint32_t pressure = 0;

    while(read8(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_OST)
        _delay_ms(1);

    ctrl_reg1.bit.ALT = 0;
    write8(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);

    ctrl_reg1.bit.OST = 1;
    write8(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);

    uint8_t sta = 0;
    while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
        sta = read8(MPL3115A2_REGISTER_STATUS);
        _delay_ms(1);
    }

    /* The pressure data is stored as a 20-bit unsigned integer with a
     * fractional part. Pressure data in Pascals.
     *
     *           7                 4 3               0
     *           +------------------------------------+
     * OUT_P_MSB |          Pressure [19:12]          | <- Integer part
     *           +------------------------------------+
     * OUT_P_CSB |          Pressure [11:4]           | <- Integer part
     *           +------------------------------------+
     * OUT_P_LSB |  Pressure [3:0]  |                 | <- Fractional part
     *           +------------------------------------+
     */

    /* We have to read 3 bytes of data from the pressure register. */
    (void)I2C_readReg(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB,
                    data, 3);

    /* OUT_P_MSB */
    pressure |= data[0];
    pressure <<= 8;

    /* OUT_P_CSB */
    pressure |= data[1];
    pressure <<= 8;

    /* OUT_P_LSB */
    pressure |= data[2];
    pressure >>= 4;

    double baro = pressure;
    baro /= 4.0;

    /* Convert Pa to hPa. */
    return baro / 100.f;
}

/**************************************************************************/
/*!
      @brief   Gets the doubleing-point altitude value
      @return altitude reading as a doubleing-point value
*/
/**************************************************************************/
double MPL3115A2_getAltitude() {
    uint8_t data[3] = {0};
    int32_t alt = 0;

    while(read8(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_OST) _delay_ms(10);

    ctrl_reg1.bit.ALT = 1;
    write8(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);

    ctrl_reg1.bit.OST = 1;
    write8(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);

    uint8_t sta = 0;
    while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
      sta = read8(MPL3115A2_REGISTER_STATUS);
      _delay_ms(1);
    }

    /* The altitude data is stored as a 20-bit signed integer with a
     * fractional part. The OUT_P_MSB (01h) and OUT_P_CSB (02h) registers
     * contain the integer part in meters and the OUT_P_LSB (03h) register
     * contains the fractional part.
     *
     *           7                 4 3               0
     *           +------------------------------------+
     * OUT_P_MSB |          Altitude [19:12]          | <- Integer part
     *           +------------------------------------+
     * OUT_P_CSB |          Altitude [11:4]           | <- Integer part
     *           +------------------------------------+
     * OUT_P_LSB |  Altitude [3:0]  |                 | <- Fractional part
     *           +------------------------------------+
     */

    /* We have to read 3 bytes of data from the altitude register. */
    (void)I2C_readReg(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB,
                    data, 3);

    /* OUT_P_MSB */
    alt |= ((uint32_t)data[0]) << 24;

    /* OUT_P_CSB */
    alt |= ((uint32_t)data[1]) << 16;

    /* OUT_P_LSB */
    alt |= ((uint32_t)data[2]) << 8;

    double altitude = alt;
    altitude /= 65536.0;
    return altitude;
}

/**************************************************************************/
/*!
      @brief   Set the local sea level barometric pressure
      @param pascal the pressure to use as the baseline
*/
/**************************************************************************/
void MPL3115A2_setSeaPressure(double pascal) {
    uint16_t bar = pascal / 2;
    uint8_t data[2] = {
        (uint8_t)(bar >> 8),
        (uint8_t)bar
    };

    (void)I2C_writeReg(MPL3115A2_ADDRESS, MPL3115A2_BAR_IN_MSB, data, 2);
}

/**************************************************************************/
/*!
      @brief   Gets the doubleing-point temperature in Centigrade
      @return temperature reading in Centigrade as a doubleing-point value
*/
/**************************************************************************/
double MPL3115A2_getTemperature() {
    uint8_t data[2] = {0};
    int16_t t = 0;

    uint8_t sta = 0;
    while (! (sta & MPL3115A2_REGISTER_STATUS_TDR)) {
      sta = read8(MPL3115A2_REGISTER_STATUS);
      _delay_ms(1);
    }

    /* The temperature data is stored as a signed 12-bit integer with a
     * fractional part. The OUT_T_MSB (04h) register contains the integer part
     * in °C and the OUT_T_LSB (05h)
     *
     *           7                 4 3               0
     *           +------------------------------------+
     * OUT_T_MSB |        Temperature [11:4]          | <- Integer part
     *           +------------------------------------+
     * OUT_T_LSB | Temperature [3:0]|                 | <- Fractional part
     *           +------------------------------------+
     */

    /* We have to read 2 bytes of data from the temperature register. */
    (void)I2C_readReg(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_TEMP_MSB,
                    data, 2);

    /* OUT_T_MSB */
    t |= ((uint16_t)data[0]) << 8;

    /* OUT_P_LSB */
    t |= ((uint32_t)data[1]);
    t >>= 4;

    /* Extend sign from 12-bit to 16-bit. */
    if (t & 0x800) {
        t |= 0xF000;
    }

    double temp = t;
    temp /= 16.0;
    return temp;
}

/**************************************************************************/
/*!
      @brief   read 1 byte of data at the specified address
      @param a the address to read
      @return the read data byte
*/
/**************************************************************************/
static uint8_t read8(uint8_t a) {
   uint8_t data = 0x00;

   /* Read one byte from address 'a'.
    * Ignore for now the return value. There should be implemented a
    * proper error handling mechanism.
    */
   (void)I2C_readReg(MPL3115A2_ADDRESS, a, &data, 1);

   return data;
}

/**************************************************************************/
/*!
      @brief   write a byte of data to the specified address
      @param a the address to write to
      @param d the byte to write
*/
/**************************************************************************/
static void write8(uint8_t a, uint8_t d) {
   /* Write one byte of data 'd' to address 'a'. */
   (void)I2C_writeReg(MPL3115A2_ADDRESS, a, &d, 1);
}
