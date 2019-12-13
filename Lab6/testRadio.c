//Radio test code

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

extern enum radio_band{FM, AM, SW};
extern volatile uint8_t STC_interrupt;

volatile enum radio_band current_radio_band = FM;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

volatile uint16_t current_fm_freq = 9990;
volatile uint16_t current_am_freq;
volatile uint16_t current_sw_freq;
uint8_t  current_volume;

//Used in debug mode for UART1
//char uart1_tx_buf[40];      //holds string to send to crt
//char uart1_rx_buf[40];      //holds string that recieves data from uart


//******************************************************************************
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){
	STC_interrupt = TRUE;
	PORTF ^= (1 << PF1);
}
/***********************************************************************/


void radio_init(){

	for(int i = 0; i < 2; i++){

		
		

	}


}


int main(){
        init_twi();

	DDRF |= (1 << PF1);
	PORTF |= (0 << PF1);

        //Setup audio output (max)
	DDRE  |= (1 << PE3);
        PORTE |= (1 << PE3);
	


        EICRB |= (1<<ISC71) | (1<ISC70);
	EIMSK |= (1<<INT7);



	_delay_ms(1000);



	sei();

	


	
	   while(1){



		_delay_ms(3000);

		radio_pwr_dwn();

		_delay_ms(3000);

		radio_init();


		

	   }
	 
}