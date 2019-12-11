// lab4.c 

//Cody McCall
// 10/11/2019

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "hd44780.h"
#ifndef HELPERS_H
#include "helpers.h"
#endif
#include "uart_functions.h"

//******************************************************************************
//							TIMER1_OVF_vect ISR
//ISR outputs sound if alarm is enabled to PD4. Sound is 5Vpp
//
ISR(TIMER1_OVF_vect){
    if(1){PORTD ^= (1<<PD4);}
}

//******************************************************************************
//							TIMER0_OVF_vect ISR
//ISR checks all inputs and reacts accordingly. Read each encoder and one button every
//cycle. Buttons are all checked with button_read variable
//
ISR(TIMER0_OVF_vect){
	ADCSRA |= (1<<ADSC);
	while(bit_is_clear(ADCSRA, ADIF)){}
	ADCSRA ^= (1<<ADIF);
	OCR2 = ~(ADC/4) + 0;

	//*****track/display time*****//
    if(!(mode & 0x07)){                             //normal time display
        secsCount++;                                //count cycles
        if(secsCount == 64){                        //0.5 seconds have passed
            segment_data[2] = (BLANK + (am_pm*3));  //turn off colon
        }
        if(secsCount == 129){                       //1 second had passed
            secs++;                                 //increment seconds
			updateTime();                           //update minutes hours, and colon
            segsumTime();                           //put time into display buffer
            secsCount = 0;                          //reset seconds
        }
    }
	if((mode == 0x04) | (mode == 0x05) | (mode == 0x06)){
		secsCount++;
		if(secsCount % 2)
        	segsumAlarm();			
        if(secsCount == 129){
            secs++;
			updateTime();
            secsCount = 0;
        }
	}
	if((mode == (1<<ARMALARM)) && (armed != TRUE)){ //arm alarm for current alarm time
		armed = TRUE;
	}
    if((mode == 0x01) | (mode == 0x02)){            //display time but pase counter
        segsumTime();
    }

    readInputs();                                   //read all inputs
}


//***********************************************************************************
uint8_t main()
{
//*****initialization*****//
DDRB = 0xFF;							//PORTB set all outputs (display decoder and SPI)
DDRE = (1<<PE3)| (1<<PE6) | (1<<PE7);	//PORTE set outputs for bits 3 (Volume), 6 (SHLD), and 7 (CLK_INH)
DDRD = (1<<PD4);						//PORTD set ouput for bit 4 (alarm sound)
DDRF &= ~(1<<PF7);
PORTF &= ~(1<<PF7);
PORTE = (1<<SH_LD) | (1<<CLK_INH);		//init encoder SH/LD and CLK_INH high
initTimers();
//Brightness control: ADC single ended
ADMUX = 0x47;				//single ended, input PF7, right adjusted, 10 bits
ADCSRA = (1 << ADEN) | (0 << ADSC) | (1<< ADPS2) | (1 << ADPS1) | (1 <<ADPS0);

uint8_t digitCount = 0;				//tracks digit to display
uint8_t trigger_hours = 0;
uint8_t trigger_mins = 0;
uint8_t trigger_secs = 0;
uint8_t trigger_am_pm = 0;
spi_init();							//initialize SPI
lcd_init();
clear_display();
sei();								//enable interrupts
hours = 1;
mins = 0;
am_pm = 1;
alarm_hours = 7;
alarm_mins = 30;
segment_data[2] = BLANK;
OCR2 = 250;
OCR3A = idle_volume;				//set inital volume
while(1){
	//*****alarm control*****//
	if(armed == 1){								//arm the alarm
		armed++;								//move to armed and trigger set
		clear_display();						//clear display
		string2lcd(lcd_armed);					//write armed to display
		trigger_hours = alarm_hours;			//set trigger hours
		trigger_mins = alarm_mins;				//set trigger minutes
		//parse trigger time to lcd
		sprintf(lcd_trigger, "%d%d:%d%d", trigger_hours/10, trigger_hours%10, trigger_mins/10, trigger_mins%10);
		string2lcd(lcd_trigger);				//write to lcd
		trigger_am_pm = alarm_am_pm;			//set trigger am/pm
		trigger_secs = 0;						//set trigger seconds
		mode &= ~(1<<ARMALARM);					//reset mode to confirm arming
	}
	if((armed) && (hours == trigger_hours) && (mins == trigger_mins) && (am_pm == trigger_am_pm) && (secs == trigger_secs)){	//condition to trigger alarm
		/*if((mode & 0x80)){
			alarm_mins = (alarm_mins + 10) % 60;
		}
		else{*/
			clear_display();
			string2lcd(lcd_beep);
			armed = FALSE;
			snooze = FALSE;
			mode &= ~(1<< SNOOZE);
			alarm = TRUE;
			trigger_hours = 0;
			trigger_mins = 0;
			trigger_secs = 0;
			trigger_am_pm = 0;
		//}
	}
	if(alarm){
		OCR3A = active_volume;
		if((mode & 0x80) && !snooze){
			clear_display();
			string2lcd(lcd_snooze);
			trigger_hours = hours;
			trigger_mins = mins;
			trigger_secs = secs + 10;
			trigger_am_pm = am_pm;
			//alarm_mins = (alarm_mins + 10) % 60;
			alarm = FALSE;
			OCR3A = idle_volume;
			armed = 2;
			mode &= ~(1<< SNOOZE);
		}
		if(mode & 0x40){
			clear_display();
			alarm = FALSE;
			OCR3A = idle_volume;
			mode ^= (0x40);
		}
	}
	

	//*****display count*****//
	if(count > 1023){count = 0;}						//keep in range 0-1023
	if(count < 0){count = 1023;}						//keep in range 0-1023
	//segsum(count);								    //load display array
	//segsumTime();                                       //load display array with time
    if(digitCount == 6){digitCount= 0;}					//reset digit count to zero
	DDRA = 0xFF;										//PORTA output mode
    _delay_us(10);
	PORTA = dec_to_7seg[segment_data[digitCount]];		//send 7 segment code to LED segments
	PORTB = (0 << PWM) | (digitCount << 4);				//send PORTB the digit to display
	digitCount++;										//update digit to display

  }//while
}//main