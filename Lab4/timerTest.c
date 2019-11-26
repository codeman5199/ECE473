#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "hd44780.h"

int tick = 0;
int alarm = 0;

ISR(TIMER1_OVF_vect){
    if(alarm){PORTD ^= (1<<PD4);}
}

uint8_t main(){
    DDRE = (1<<PE3);
    DDRD = (1<<PD4);
    //Sound generation: normal mode no prescale, flip PE3 on OVF interrupt
    TCCR1A = 0x00;                  //normal mode
    TCCR1B = (1<<CS10); 	        //use no prescale clk/1
    TCCR1C = 0x00;                  //no force compare
    TIMSK = (1<<TOIE1);             //enable OVF interrupt

    //Volume control: fast pwm, set on match, clear at top, ICR1 holds TOP 
    TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31);   //use ICR1 as source for TOP, use clk/1
    TCCR3B |= (1<<WGM33) | (1<< WGM32) | (1<<CS30); //no forced compare 
    TCCR3C = 0x00;                                
    
    OCR3A = 0x00070; //set   at 0xC000
    ICR3  = 0x000FF; //clear at 0xF000
    
    sei();
    while(1){
        /*if(TIFR & (1<<TOV1)){
            TIFR |= (1<<TOV1);
            PORTD ^= (1<<PD4);
        }*/
    }
}