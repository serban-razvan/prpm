PORT ?= /dev/tty.usbserial-DN00S3BK
CFLAGS=-Wl,-u,vfprintf -lprintf_flt -lm
all: lab5.hex

program: upload

lab5.elf: lab5.c spi.c usart.c i2c_master.c MPL3115A2.c LSM9DS0.c ST7735R_TFT.c game.c
	avr-g++ $(CFLAGS) -mmcu=atmega324p -DF_CPU=16000000 -Wall -Os -o $@ $^

lab5.hex: lab5.elf
	avr-objcopy -j .text -j .data -O ihex lab5.elf lab5.hex
	avr-size lab5.elf

upload: lab5.hex
	avrdude -c arduino -P $(PORT) -b 57600 -p atmega324p -U flash:w:$<:a

clean:
	rm -rf lab5.elf lab5.hex

.PHONY: all clean program upload

