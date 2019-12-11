
#ifndef HELPERS_H
#include "helpers.h"
#endif

//******************************************************************************
//							spi_init
//Initialize SPI module in master mode, with low polarity, and MSB lsb format.
//Runs at  i/o clock/2
//
void spi_init(){
	DDRF  |= 0x08;  //port F bit 3 is enable for LCD
	PORTF &= 0xF7;  //port F bit 3 is initially low
	DDRB |= 0x07;						//set MISO, MOSI, SCK, and SS to output
	SPCR = (1<<SPE) | (1<<MSTR);		//SPI enable, master mode, low polarity, MSB lsb
	SPSR = (1<<SPI2X);					//run at i/o clock/2
}

//******************************************************************************
//							write_bargraph
//Sends value to bargraph via SPI. Pass in data value to be written to bargraph.
//Note MSB corresponds to bottom of bargraph.
//
void write_bargraph(uint8_t data){
	SPDR = data;
	while(bit_is_clear(SPSR,SPIF)) {}
	PORTB |= 1;
	PORTB &= 0b11111110;
}

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t chk_buttons(uint8_t button) {
	static uint16_t state[8] = {0,0,0,0,0,0,0,0}; 											//holds present state, each button has it's own state
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;			//shift in state of button pin to appropriate state variable
	if (state[button] == 0xF000) return 1;													//if button has been held for 12 cycles, return 1
	return 0;																				//else return 0
}

//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//
void segsum(uint16_t sum) {
	uint8_t numDigits= 0;
	uint8_t ones,tens,hundreds,thousands;
	if(sum == 0){ 								//special case to display 0
	segment_data[4] = BLANK;
	segment_data[3] = BLANK;
	segment_data[2] = BLANK;
	segment_data[1] = BLANK;
	segment_data[0] = 0;
	return;
	}
  //determine how many digits there are 
	if(sum/1000 != 0){numDigits++;}     		//check if there is anything in the thousands place
	if(sum/100 != 0){numDigits++;}				//check hundreds place
	if(sum/10 != 0){numDigits++;}				//check tens place
	if(sum/1 != 0){numDigits++;}				//check ones place
  //break up decimal sum into 4 digit-segments
	thousands = sum/1000;						//thounsands plave value (thanks to integer division)
	hundreds = (sum%1000)/100;					//hundreds place value, mod removes the thousands place
	tens = (((sum%1000)%100)/10);				//tens place value, mod removes hundreds and thousands place
	ones = (((sum%1000)%100)%10);				//ones place value, modding out everything but the ones place
  //store values in correct indexes
	segment_data[4] = thousands;
	segment_data[3] = hundreds;
	segment_data[2] = BLANK;
	segment_data[1] = tens;
	segment_data[0] = ones;
  //blank out leading zero digits stored valule 10 = blank
	if(numDigits < 4){segment_data[4] = BLANK;} 
	if(numDigits < 3){segment_data[3] = BLANK;}
	if(numDigits < 2){segment_data[1] = BLANK;}
	if(numDigits < 1){segment_data[0] = BLANK;}
  //now move data to right place for misplaced colon position
}//segment_sum

//******************************************************************************
//									updateTime
//update the current minutes, seconds, and hours
void updateTime(){
    if(secs >=60){                              //add minute after 60 seconds
        mins++;
        secs = 0;                               //reset seconds
    }
    if(mins >= 60){                             //add hour after 60 minutes
        hours++;
        mins = 0;                               //reset minutes
    }
    if(hours >=13){                             //overflow hour to 1 after 12 hours (12hour format)
		am_pm ^= 1;
        hours = 1;
    }
}

//***********************************************************************************
//                                   segsumTime                                    
//converts time into values to be displayed for the current time. Additionally, deals
//with overflow at limites                 
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//
void segsumTime(){
    //store values in correct indexes
	segment_data[4] = hours/10;
	segment_data[3] = hours%10;
    segment_data[2] = COLON_ON + am_pm;
	segment_data[1] = mins/10;
	segment_data[0] = mins%10;
}

//***********************************************************************************
//                                   segsumTime                                    
//converts time into values to be displayed for the current time. Additionally, deals
//with overflow at limites                 
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//
void segsumAlarm(){
    //store values in correct indexes
	segment_data[4] = alarm_hours/10;
	segment_data[3] = alarm_hours%10;
    segment_data[2] = COLON_ON + alarm_am_pm;
	segment_data[1] = alarm_mins/10;
	segment_data[0] = alarm_mins%10;
}

//***********************************************************************************

//***********************************************************************************
//                                   readButton
//read selected button for full debounce cycle. Button selected by changing button param.
//
int readButton(uint8_t button){
	//make PORTA an input port with pullups 
	PORTA = 0xFF;				//turn off display and prep for pullup values
	asm volatile("nop");
	asm volatile("nop");
	DDRA = 0x00;
  	//enable tristate buffer for pushbutton switches
	PORTB = (1 << PWM) | (BTTN_EN << 4);
  	//now check button 1 and change mode if needed
	for(uint8_t delay = 0; delay < 20; delay++){
		if(chk_buttons(button)){
			return TRUE;
		}
	}
	return FALSE;
}

//***********************************************************************************
//                                   quad1Read
//read encoder 1 and return pulse for each cw or ccw tick on encoder. Populate quad1_prev
//and quad1_current before calling funciton. 
// TRUTH TABLE: 	| Direction	| return val |
//					| no turn	|	  0		 |
//					| ccw turn	|	  1		 |
//					| cw turn	|	  2		 |
//
int quad1Read(uint8_t quad1_prev, uint8_t quad1_current){
	static int8_t count1;									//counter for quadrature edges
	if(quad1_current == quad1_prev){						//no turn case
		return 0;											//return 0 for no turn
	}
	switch(quad1_prev){										//compare previous measured quad value with current one
		case 0b00:
			if(quad1_current == 0b10){count1++;}			//ccw
			else if(quad1_current == 0b01){count1--;}		//cw
			break;
		case 0b01:
			if(quad1_current == 0b00){count1++;}			//ccw
			else if(quad1_current == 0b11){count1--;}		//cw
			break;
		case 0b10:
			if(quad1_current == 0b11){count1++;}			//ccw
			else if(quad1_current == 0b00){count1--;}		//cw
			break;
		case 0b11:
			if(quad1_current == 0b01){count1++;}			//ccw
			else if(quad1_current == 0b10){count1--;}		//cw
			break;
	}	
	if(count1 == 4){										//count 4 edges for one tick
		count1 = 0;											//reset counter
		return 1;											//return 1 for ccw
	}
	else if(count1 == -4){									//count 4 edges for one tick
		count1 = 0;											//reset counter
		return 2;											//return 2 for cw
	}
	return 0;
}

//***********************************************************************************
//                                   quad2Read
//read encoder 2 and return pulse for each cw or ccw tick on encoder. Populate quad2_prev
//and quad2_current before calling funciton. 
// TRUTH TABLE: 	| Direction	| return val |
//					| no turn	|	  0		 |
//					| ccw turn	|	  1		 |
//					| cw turn	|	  2		 |
//
int quad2Read(uint8_t quad2_prev, uint8_t quad2_current){
	static int8_t count2;
	if(quad2_current == quad2_prev){						//no turn case
		return 0;											//return 0 for no turn
	}
	switch(quad2_prev){										//compare previous measured quad value with current one
		case 0b00:
			if(quad2_current == 0b10){count2++;}			//ccw
			else if(quad2_current == 0b01){count2--;}		//cw
			break;
		case 0b01:
			if(quad2_current == 0b00){count2++;}			//ccw
			else if(quad2_current == 0b11){count2--;}		//cw
			break;
		case 0b10:
			if(quad2_current == 0b11){count2++;}			//ccw
			else if(quad2_current == 0b00){count2--;}		//cw
			break;
		case 0b11:
			if(quad2_current == 0b01){count2++;}			//ccw
			else if(quad2_current == 0b10){count2--;}		//cw
			break;
	}	
	if(count2 == 4){										//count 4 edges for one tick
		count2 = 0;											//reset counter
		return 1;											//return 1 for ccw
	}
	else if(count2 == -4){									//count 4 edges for one tick
		count2 = 0;											//reset counter
		return 2;											//return 2 for cw
	}
	return 0;
}

//***********************************************************************************
//                                   incSetTime
//increment minutes or hours of time depending on current mode. Mode is determined by mode variable
//	TRUTH TABLE:	| mode val	| operation  |
//					|  0x01 	|	+/-mins	 |
//					|  0x02 	|	+/-hours |
void incSetTime(){
    switch(mode){							//change minutes or hours depending on mode
		case (1<<SETMINS):					//set minutes mode
			mins++;							//increment minutes
            if(mins == 60){					//overflow
                mins = 0;
            }
			break;
		case (1<<SETHOURS):					//change hours mode
			hours++;						//increment hours
            if(hours == 13){				//overflow
				am_pm ^= 1;
                hours = 1;
            }
			break;
		case (1<<DISP_TOGGLE) | (1<<SETMINS):	//set alarm minutes mode
			alarm_mins++;					//increment minutes
			if(alarm_mins == 60){			//underflow back to 0
                alarm_mins = 0;
            }
			break;
		case (1<<DISP_TOGGLE) | (1<<SETHOURS):	//set alarm hours mode
			alarm_hours++;					//increment minutes
			if(alarm_hours == 13){			//underflow back to 1
				alarm_am_pm ^= 1;
                alarm_hours = 1;
            }
			break;
	}
}

//***********************************************************************************
//                                   incSetTime
//decrements minutes or hours of time depending on current mode. Mode is determined by mode variable
//	TRUTH TABLE:	| mode val	| operation  |
//					|  0x01 	|	+/-mins	 |
//					|  0x02 	|	+/-hours |
void decSetTime(){
	switch(mode){						//change minutes or hours depending on mode
		case 0x01:						//set minutes mode
            if(mins == 0){				//underflow back to 0
                mins = 60;
            }
			mins--;						//decrement minutes
			break;
		case 0x02:						//set hours mode
			if(hours == 1){				//underflow back to 1
				am_pm ^= 1;
                hours = 13;
            }
			hours--;					//decrement minutes
			break;
		case 0x05:						//set hours mode
			if(alarm_mins == 0){		//underflow back to 0
                alarm_mins = 60;
            }
			alarm_mins--;				//decrement minutes
			break;
		case 0x06:						//set hours mode
			if(alarm_hours == 1){		//underflow back to 1
				alarm_am_pm ^= 1;
                alarm_hours = 13;
            }
			alarm_hours--;				//decrement minutes
			break;
	}
}

void incdec_vol(uint8_t dir){
	static uint8_t count = 0x07;
	switch(dir){
		case 0:
			if(count < 0x0F){
				count++;
				//write_bargraph(count);
				active_volume = (count << 4);
			}
		break;
		case 1:
			if(count > 0x04){
				count--;
				//write_bargraph(count);
				active_volume = (count << 4);
			}
		break;

	}
}

void readInputs(){
    //*****check inputs*****//
	PORTE = 0b01000000;								//toggle SDLD and CLK_INH
    SPDR = 0x00;									//write garbage over SPI
	while(bit_is_clear(SPSR, SPIF)){}				//spin until transmission complete
	quad1_prev = quad1_current;						//set previous value to current
	quad2_prev = quad2_current;						//set previous value to current
	data = SPDR;									//read new current value
	quad1_current = (data & 0x03);					//mask out all bits except 0 and 1
	quad2_current = ((data & 0x0C)>>2);				//mask out all bits except 2 and 3, then shift right by 2
	turn = quad1Read(quad1_prev, quad1_current);	//eval state of encoder 1
	if(turn == 1){incSetTime();}					//if turning ccw, inc
	if(turn == 2){decSetTime();}					//if turning cw, dec
	turn = quad2Read(quad2_prev, quad2_current);	//eval state of encoder 2
	if(turn == 1){incdec_vol(0);}					//if turning ccw, inc
	if(turn == 2){incdec_vol(1);}					//if turning cw, dec
	PORTE = 0b10000000;								//toggle SDLD and CLK_INH
	
    //read buttons
	if(readButton(button_read)){					//eval buttons
		mode ^= (1<<button_read);					//toggle bit 2 of mode
	}
	write_bargraph(mode);							//update mode
    button_read++;									//check next button
    if(button_read >=8)								//reset to 0 after all buttons
        button_read = 0;
}

void initTimers(){
    //Timer Int vectors used: TIMER0_OVF, TIMER1_OVF
    TIMSK |= (1<<TOIE0) | (1<<TOIE1);   	//enable interrupts
    //Input control: normal mode, ext osc, no prescale
    ASSR  |= (1<<AS0);                  	//use ext oscillator
    TCCR0 |= (1<<CS00);  	            	//normal mode, no prescale
    //Brightness control: fast PWM, 1024 prescale, ouput on OC2
    TCCR2 |= (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS20) | (1<<CS22);   //fast PWM mode, 1024 prescale
    //Sound generation: normal mode no prescale, flip PE3 on OVF interrupt
    TCCR1A = 0x00;                  					//normal mode
    TCCR1B = (1<<CS10); 	        					//use no prescale clk/1
    TCCR1C = 0x00;                  					//no force compare
    //Volume control: fast pwm, set on match, clear at top, ICR1 holds TOP 
    TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31);   //use ICR1 as source for TOP, use clk/1
    TCCR3B |= (1<<WGM33) | (1<< WGM32) | (1<<CS30); 	//no forced compare 
    TCCR3C = 0x00;                                    	//
    ICR3  = 0x00FF; 									//clear at 0x00FF
}