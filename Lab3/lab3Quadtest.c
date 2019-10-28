//Quadrature encoder testing

#define F_CPU 16000000 // cpu speed in hertz 
#include <avr/io.h>
#include <util/delay.h>

uint8_t quad_prev = 0;
uint8_t quad_current = 0;
int count = 0;

void quadRead(){
	switch(quad_prev){								//compare previous measured quad value with current one
		case quad_current:							//if current and past input match, no turn
			break;
		case 0b00:
			if(quad_current == 0b10){count++;}				//cw
			else if(quadcurrent == 0b01){count--;}			//ccw
			break;
		case 0b01:
			if(quad_current == 0b00){count++;}				//cw
			else if(quadcurrent == 0b11){count--;}			//ccw
			break;
		case 0b10:
			if(quad_current == 0b11){count++;}				//cw
			else if(quadcurrent == 0b00){count--}			//ccw
			break;
		case 0b11:
			if(quad_current == 0b01){count++;}				//cw
			else if(quadcurrent == 0b10){count--}			//ccw
			break;
	}	
}

uint8_t main(){
	quadRead();
	
}
