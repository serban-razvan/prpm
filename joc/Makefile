PORT ?= COM12

all: program

program: upload

proiect.elf: main.c usart.c i2chw/twimaster.c adxl345/adxl345.c
	avr-g++ -mmcu=atmega324p -DF_CPU=16000000 -Wall -Os -o $@ $^

proiect.hex: proiect.elf
	avr-objcopy -j .text -j .data -O ihex proiect.elf proiect.hex
	avr-size proiect.elf

upload: proiect.hex
	../bootloadHID -r proiect.hex

clean:
	rm -rf proiect.elf proiect.hex

.PHONY: all clean program upload

