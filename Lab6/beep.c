#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "uart_functions.h"
#include "si4734.h"
#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 		1
#define FALSE 		0

ISR(TIMER1_OVF_vect){
    PORTD ^= (1<<PD4);
}

uint8_t main(){

    while(1);
}