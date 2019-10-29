
#define F_CPU 16000000 // cpu speed in hertz 
#include <avr/io.h>
#include <util/delay.h>

uint8_t data = 0b00000010;

void spi_init(){
	DDRB = 0X07;
	SPCR = (1<<SPE) | (1<<MSTR);
	SPSR = (1<<SPI2X);
}

uint8_t main(){
	spi_init();
	SPDR = data;
	while(bit_is_clear(SPSR, SPIF)){

	}
	PORTB = 0b00000001;
	PORTB = 0;
}
