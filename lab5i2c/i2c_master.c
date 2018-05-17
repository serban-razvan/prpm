#include <avr/io.h>
#include <util/twi.h>

#include "I2C_master.h"

/* The I2C buffer is enabled only during a transaction. */

#define I2C_BUF_PORT      DDRA
#define I2C_BUF_DDR       PORTA
#define I2C_BUF_PIN       PA6
#define I2C_BUF_INIT()    (I2C_BUF_DDR |= (1 << I2C_BUF_PIN))
#define I2C_BUF_ENABLE()  (I2C_BUF_PORT |= (1 << I2C_BUF_PIN))
#define I2C_BUF_DISABLE() (I2C_BUF_PORT &= ~(1 << I2C_BUF_PIN))

#define I2C_READ    0x01
#define I2C_WRITE   0x00
#define F_SCL       400000UL // SCL frequency 400KHz
#define Prescaler   1
#define TWBR_VAL    ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

void I2C_init(void)
{
    // Init the I2C buffer.
    I2C_BUF_INIT();

    // Set the I2C frequency.
	  TWBR = (uint8_t)TWBR_VAL;
}

uint8_t I2C_start(uint8_t address)
{
    // We're starting an I2C transaction - enable the I2C buffer.
    I2C_BUF_ENABLE();

    // reset TWI control register
    TWCR = 0;
    // transmit START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    // wait for end of transmission

    while( !(TWCR & (1<<TWINT)) );

    // check if the start condition was successfully transmitted
    if((TWSR & 0xF8) != TW_START){ return 1; }

    // load slave address into data register
    TWDR = address;
    // start transmission of address
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );

    // check if the device has acknowledged the READ / WRITE mode
    uint8_t twst = TW_STATUS & 0xF8;
    if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

    return 0;
}

uint8_t I2C_write(uint8_t data)
{
    // load data into data register
    TWDR = data;
    // start transmission of data
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );

    if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }

    return 0;
}

uint8_t I2C_read_ack(void)
{
    // start TWI module and acknowledge data after reception
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    // return received data from TWDR
    return TWDR;
}

uint8_t I2C_read_nack(void)
{
    // start receiving without acknowledging reception
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    // return received data from TWDR
    return TWDR;
}

void I2C_stop(void)
{
    // transmit STOP condition
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

    // I2C transaction is finished - disable the buffer.
    I2C_BUF_DISABLE();
}

uint8_t I2C_transmit(uint8_t address, uint8_t* data, uint16_t length)
{
    /* Enable the I2C buffer. */

    if (I2C_start(address | I2C_WRITE))
          return 1;

    for (uint16_t i = 0; i < length; i++)
    {
      if (I2C_write(data[i]))
              return 1;
    }

    I2C_stop();

    return 0;
}

uint8_t I2C_receive(uint8_t address, uint8_t* data, uint16_t length)
{
    if (I2C_start(address | I2C_READ))
          return 1;

    for (uint16_t i = 0; i < (length-1); i++)
    {
      data[i] = I2C_read_ack();
    }

      data[(length-1)] = I2C_read_nack();
    I2C_stop();

    return 0;
}

uint8_t I2C_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
    if (I2C_start(devaddr | I2C_WRITE))
          return 1;

    I2C_write(regaddr);

    for (uint16_t i = 0; i < length; i++)
    {
      if (I2C_write(data[i]))
              return 1;
    }

    I2C_stop();

    return 0;
}

uint8_t I2C_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
    if (I2C_start(devaddr))
          return 1;

    I2C_write(regaddr);

    if (I2C_start(devaddr | I2C_READ))
          return 1;

    for (uint16_t i = 0; i < (length-1); i++)
    {
      data[i] = I2C_read_ack();
    }

      data[(length-1)] = I2C_read_nack();
    I2C_stop();

    return 0;
}

