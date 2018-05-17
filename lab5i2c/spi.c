#include <avr/io.h>

#include "spi.h"

void SPI_init(void)
{
    // set SPI direction: MOSI and SCK as output, MISO as input
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << PB4);
    SPI_DDR &= ~(1 << SPI_MISO);

    SPSR |= (1 << SPI2X);			          // f_osc / 2

    // initialize SPI
    SPCR =  (1 << SPE) |			          // enable
            (1 << MSTR) |			          // master mode
            (0 << DORD) |			          // MSB first
            (0 << CPOL) | (0 << CPHA) |	// mode 0
            (0 << SPR0);			          // f_osc / 2
}

#include <stdio.h>
uint8_t SPI_exchange(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1 << SPIF)))
  {
    printf("waiting\n");
  };
	return SPDR;
}

