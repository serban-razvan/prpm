/*!
 * @file Adafruit_MPL3115A2.h
 *
 * This is part of Adafruit's MPL3115A2 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit MPL3115A2 breakout:
 * https://www.adafruit.com/products/1893
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include <avr/io.h>
#include <util/twi.h>

/* Default I2C address */
#define MPL3115A2_ADDRESS    (0x60 << 1)

/* ID-ul senzorului. */
#define MPL3115A2_WHOAMI_VAL 0xC4

/*
 * Functia initializeaza senzorul de presiune.
 *
 * @return - true on success, false otherwise
 */
bool MPL3115A2_init();

/*
 * Functia citeste valoarea presiunii atmosferice.
 *
 * @return - valoarea presiunii atmosferice (in pascali)
 */
double MPL3115A2_getPressure(void);

/*
 * Functia calculeaza altitudinea pe baza presiunii atmosferice curente.
 *
 * @return - valoarea altitudinii (in metri)
 */
double MPL3115A2_getAltitude(void);

/*
 * Functia citeste temperatura curenta.
 *
 * @return - valoarea temperaturii (in grade celsius)
 */
double MPL3115A2_getTemperature(void);

/*
 * Functia seteaza nivelul de referinta al presiunii necesar pentru calcularea
 * altitudinii.
 *
 * @return - None.
 */
void MPL3115A2_setSeaPressure(double pascal);
