#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <avr/io.h>

void    I2C_init(void);
uint8_t I2C_start(uint8_t address);
uint8_t I2C_write(uint8_t data);
uint8_t I2C_read_ack(void);
uint8_t I2C_read_nack(void);
uint8_t I2C_transmit(uint8_t address, uint8_t* data, uint16_t length);
uint8_t I2C_receive(uint8_t address, uint8_t* data, uint16_t length);
uint8_t I2C_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
uint8_t I2C_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
void    I2C_stop(void);

#endif // I2C_MASTER_H
