//Quadrature encoder testing

#define F_CPU 16000000 // cpu speed in hertz 
#define SDLD	6
#define CLKINH	7
#include <avr/io.h>
#include <util/delay.h>

uint8_t quad2_prev = 0;
uint8_t quad2_current = 0;
int count = 0;
uint8_t data = 0x00;

//******************************************************************************
//							spi_init
//Initialize SPI module in master mode, with low polarity, and MSB lsb format.
//Additionally run at  i/o clock/2
//
void spi_init(){
	DDRB |= 0x07;						//set MISO, MOSI, SCK, and SS to output
	SPCR = (1<<SPE) | (1<<MSTR);		//SPI enable, master mode, low polarity, MSB lsb
	SPSR = (1<<SPI2X);					//run at i/o clock/2
}

int quad1Read(uint8_t quad1_prev, uint8_t quad1_current){
	static int8_t count1;
	if(quad1_current == quad1_prev){
		return 0;
	}
	switch(quad1_prev){								//compare previous measured quad value with current one
		case 0b00:
			if(quad1_current == 0b10){count1++;}				//cw
			else if(quad1_current == 0b01){count1--;}			//ccw
			break;
		case 0b01:
			if(quad1_current == 0b00){count1++;}				//cw
			else if(quad1_current == 0b11){count1--;}			//ccw
			break;
		case 0b10:
			if(quad1_current == 0b11){count1++;}				//cw
			else if(quad1_current == 0b00){count1--;}			//ccw
			break;
		case 0b11:
			if(quad1_current == 0b01){count1++;}				//cw
			else if(quad1_current == 0b10){count1--;}			//ccw
			break;
	}	
	PORTB = count1<<4;
	if(count1 == 4){
		count1 = 0;	
		return 1;
	}
	else if(count1 == -4){
		count1 = 0;	
		return 2;
	}
}
/*
int quad2Read(){
	static int8_t count2 = 0;
	if(quad2_current == quad2_prev){
		return 0;
	}
	switch(quad2_prev){								//compare previous measured quad value with current one
		case 0b00:
			if(quad2_current == 0b10){count2++;}				//cw
			else if(quad2_current == 0b01){count2--;}			//ccw
			break;
		case 0b01:
			if(quad2_current == 0b00){count2++;}				//cw
			else if(quad2_current == 0b11){count2--;}			//ccw
			break;
		case 0b10:
			if(quad2_current == 0b11){count2++;}				//cw
			else if(quad2_current == 0b00){count2--;}			//ccw
			break;
		case 0b11:
			if(quad2_current == 0b01){count2++;}				//cw
			else if(quad2_current == 0b10){count2--;}			//ccw
			break;
	}	
	if(count2 == 4){
		count2 = 0;	
		return 1;
	}
	else if(count2 == -4){
		count2 = 0;	
		return 2;
	}
}*/

uint8_t main()
{
uint8_t quad1_prev = 0;
uint8_t quad1_current = 0;
int turn = 0;
spi_init();
DDRB |= 0xFF;
DDRE = 0xFF;	
PORTE = 0b11000000;
while(1){
	//to read
	//sd/ld low
	//clk_inh low
	PORTE = 0b01000000;
	//spi garb
	SPDR = count;
	while(bit_is_clear(SPSR, SPIF)){}
	PORTB |= 1;
	PORTB |= 0;
	quad1_prev = quad1_current;
	data = SPDR;
	quad1_current = (data & 0x03);
	turn = quad1Read(quad1_prev, quad1_current);
	if(turn == 1){count++;}
	if(turn == 2){count--;}
	//count = count + quad1Read();
	//sd/ld high
	//clk_inh high
	PORTE = 0b10000000;
	
	//display on bargraph
	//PORTB = quad1_current<<4;
	}
}
