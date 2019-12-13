// lab4.c 

//Cody McCall
// 10/11/2019

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 		1
#define FALSE 		0
#define BLANK		10			//blank digit is represented by the value ten
#define COLON_ON	11			//colon on is represented by the value eleven
#define PWM			7			//pwm pin 7 on PORTB
#define BTTN_EN		7			//decoder pin cpnnected to enable of tristate buffer
#define DISABLE		5			//unused decoder value for disabling
#define SH_LD		6			//SH/LD pin 6 on PORTE
#define CLK_INH		5			//CLK_INH pin 5 on PORTE
#define SETMINS		0
#define	SETHOURS	1
#define DISP_TOGGLE	2
#define ARMALARM	3
#define SNOOZE		7
#define TEMP_CMD	'r'
#define DISP_RADIO	4
#define STATION1	9990
#define STATION2	9910
#define STATION3	9570
#define STATION4	9450
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


uint8_t quad1_prev = 0;				//state values for encoders 1 and 2
uint8_t quad1_current = 0;			//
uint8_t quad2_prev = 0;				//
uint8_t quad2_current = 0;			//
uint8_t snooze = FALSE;
uint8_t armed = FALSE;
uint8_t mode = 0;					//holds current inc/dec mode
uint8_t turn = 0;					//temp variable for turn direction
uint8_t data = 0;					//temp valriable for read SPI data
uint8_t alarm_hours, alarm_mins;    //variable for alarm hours and minutes
uint8_t hours, mins;                //variable for hours and minutes
uint8_t am_pm = 0; 					//state of time, AM or PM. 0 for AM, 1 for PM
uint8_t alarm_am_pm = 0; 			//state of time, AM or PM. 0 for AM, 1 for PM
uint8_t secs = 0; 
uint8_t secsCount = 0;
uint8_t blink = 0;
static int count = 1;				//count displayed on seven seg
uint8_t alarm = FALSE;				//state of alarm
uint8_t active_volume = 0x70;		//active volume
uint8_t idle_volume = 0xF0;			//idle volume
uint8_t temp_flag;
char rx_buf;
uint8_t rx_count = 0;
uint16_t radio_freq = 9990;
uint16_t prev_fm_freq = 9990;
uint8_t radio_state = 0;


char lcd_armed[16] = {'A', 'R', 'M', 'E', 'D', ' '};  //holds string to send to lcd 
char lcd_beep[16] = {'B', 'E', 'E', 'P'};  //holds string to send to lcd  
char lcd_snooze[16] = {'S', 'N', 'O', 'O', 'Z', 'E'};  //holds string to send to lcd 
char lcd_trigger[16];
char lcd_temp_local[16];
char lcd_temp_remote[16] = {'0', '0', 'C'};

uint8_t lm73_write_buf[2];
uint8_t lm73_read_buf[2];
//******************************************************************************
//							button_read
//flag to control which button is being polled at any time in the ISR
//this is only to be edited within the TIMER0_OVF_vect ISR.
//
static uint8_t button_read = 0;	

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[24] ={
0b11000000,		//0
0b11111001,		//1
0b10100100,		//2
0b10110000,		//3
0b10011001,		//4
0b10010010,		//5
0b10000010,		//6
0b11111000,		//7
0b10000000,		//8
0b10011000,		//9
0b11111111,		//10 or blank (AM)
0b11111100,     //11 or colon on (AM)
0b11111000,     //12 or colon on (PM)
0b11111011, 	//13 or blank (PM)
0b01000000,		//0.
0b01111001,		//1.
0b00100100,		//2.
0b00110000,		//3.
0b00011001,		//4.
0b00010010,		//5.
0b00000010,		//6.
0b01111000,		//7.
0b00000000,		//8.
0b00011000,		//9.
};


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
	segment_data[1] = tens + 14;
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

/*
void segsumFreq(){
    //store values in correct indexes
	segment_data[4] = radio_freq/1000;
	segment_data[3] = (radio_freq%1000)/100;
    segment_data[2] = BLANK//COLON_ON + alarm_am_pm;
	segment_data[1] = (radio_freq%100)/10;
	segment_data[0] = radio_freq%1000;
}*/

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
		case (1<<DISP_RADIO):
			if(radio_freq != 10790){
				radio_freq = radio_freq + 20;
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
		case (1<<DISP_RADIO):
			if(radio_freq != 8810){
				radio_freq = radio_freq - 20;
			}
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

void radio_start(){
	PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	PORTE |=  (1<<PE2); //hardware reset Si4734 
	_delay_us(200);     //hold for 200us, 100us by spec         
	PORTE &= ~(1<<PE2); //release reset 
	_delay_us(30);      //5us required because of my slow I2C translators I suspect
	//Si code in "low" has 30us delay...no explaination
	DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

	fm_pwr_up(); //powerup the radio as appropriate
	current_fm_freq = radio_freq; //arg2, arg3: 99.9Mhz, 200khz steps
	fm_tune_freq(); //tune radio to frequency in current_fm_freq
	OCR3A = active_volume;
}

//******************************************************************************
//							TIMER1_OVF_vect ISR
//ISR outputs sound if alarm is enabled to PD4. Sound is 5Vpp
//
ISR(TIMER1_OVF_vect){
    if(alarm){PORTD ^= (1<<PD4);}
}

ISR(INT7_vect){
	STC_interrupt = TRUE;
}

ISR(USART0_RX_vect){
    static uint8_t rx_count = 0;
	rx_buf = UDR0;
	if(rx_buf == 'C'){
		lcd_temp_remote[rx_count] = rx_buf;
		rx_count = 0;
	}
	else{
		lcd_temp_remote[rx_count] = rx_buf;
		rx_count++;
	}
	//write_bargraph(0xFF);
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
    if(!(mode & 0x07)){                    //normal time display
        secsCount++;
        if(secsCount == 64){
            segment_data[2] = (BLANK + (am_pm*3));
        }
        if(secsCount == 129){
            secs++;
			updateTime();
            segsumTime();
            temp_flag = 1;
			//uart_putc('r');
            secsCount = 0;
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
	//*****radio controls*****//
	if(!(mode & (1<<DISP_RADIO))){
		if(radio_state == 2){
			radio_state = 3;			//shut radio
		}
	}
	if((mode & (1<<DISP_RADIO)) && !(alarm)){
		if(radio_state == 0){
			radio_state = 1;			//start up radio
		}
		segsum(radio_freq/10);
		segment_data[1] = segment_data[1] & 0b01111111;
		OCR3A = active_volume;
		current_fm_freq = radio_freq; //arg2, arg3: 99.9Mhz, 200khz steps
		//fm_tune_freq();
		//station presets:
	switch(mode){
		case (1<<DISP_RADIO) | (0x01):
			radio_freq = STATION1;
			mode = (1<<DISP_RADIO);
		break;
		case (1<<DISP_RADIO) | (0x02):
			radio_freq = STATION2;
			mode = (1<<DISP_RADIO);
		break;
		case (1<<DISP_RADIO) | (0x04):
			radio_freq = STATION3;
			mode = (1<<DISP_RADIO);
		break;
		case (1<<DISP_RADIO) | (0x08):
			radio_freq = STATION4;
			mode = (1<<DISP_RADIO);
		break;
	}
	}
	

	if((mode == (1<<ARMALARM)) && (armed != TRUE)){
		armed = TRUE;
	}
    if((mode == 0x01) | (mode == 0x02)){
        segsumTime();
    }

	//*****check inputs*****//
	//PORTE = 0b11000000;							//dont clober
	PORTE &= ~(1<<CLK_INH);							//toggle CLK_INH
	PORTE |= (1<<SH_LD);							//toggle SDLD
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
	//PORTE = 0b00100000;							//dont clober
	PORTE ^= (1<<CLK_INH) | (1<<SH_LD);				//toggle SDLD and CLK_INH
	
    //read buttons
	if(readButton(button_read)){					//eval buttons
		mode ^= (1<<button_read);					//toggle bit 2 of mode
	}
	write_bargraph(mode);							//update mode
    button_read++;									//check next button
    if(button_read >=8)								//reset to 0 after all buttons
        button_read = 0;
}

//Operation Modes:
// 0x01;

//***********************************************************************************
uint8_t main()
{
//*****initialization*****//
DDRB = 0xFF;							//PORTB set all outputs (display decoder and SPI)
DDRE = (1<<PE2)|(1<<PE3)| (1<<PE6) | (1<<PE5);	//PORTE set outputs for bits 3 (Volume), 6 (SHLD), and 5 (CLK_INH)
DDRD = (1<<PD4);						//PORTD set ouput for bit 4 (alarm sound)
DDRF &= ~(1<<PF7);
PORTF &= ~(1<<PF7);
PORTE = (1<<SH_LD) | (1<<CLK_INH);		//init encoder SH/LD and CLK_INH high
PORTE |= (1 << PE2); //radio reset is on at powerup (active high)
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
//Brightness control: ADC single ended
ADMUX = 0x47;				//single ended, input PF7, right adjusted, 10 bits
ADCSRA = (1 << ADEN) | (0 << ADSC) | (1<< ADPS2) | (1 << ADPS1) | (1 <<ADPS0);

EICRB |= (1<<ISC71) | (1<ISC70);
EIMSK |= (1<<INT7);

uint8_t digitCount = 0;				//tracks digit to display
uint8_t trigger_hours = 0;
uint8_t trigger_mins = 0;
uint8_t trigger_secs = 0;
uint8_t trigger_am_pm = 0;
uint16_t tempval;
spi_init();							//initialize SPI
lcd_init();                         //initialize LCD
init_twi();                         //initalize TWI
lm73_temp_init();                   //initalize lm73 temp sensor
uart_init();						//initalize uart with interrupts
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
DDRE  |= (1 << PE2); //Port E bit 2 is active high reset for radio 
PORTE |= (1 << PE2); //radio reset is on at powerup (active high)
/*radio_start();
_delay_ms(500);
radio_pwr_dwn();*/
while(1){
	if(temp_flag == 1){
		tempval = lm73_temp_read();
        set_cursor(2, 0);								//note to self: placing this after the converstaion funcion call causes bug where tens place of temp gets cut off  
		lm73_temp_convert(lcd_temp_local, tempval, 0);//read local temperature
        //update lcd
        string2lcd(lcd_temp_local);
        set_cursor(2, 6);
        string2lcd(lcd_temp_remote);
		//send command for remote temperature
		
        temp_flag = 0;
		uart_putc('r');
    }
	//*****brightness control*****//
	/*
	ADCSRA |= (1<<ADSC);
	while(bit_is_clear(ADCSRA, ADIF)){}
	ADCSRA ^= (1<<ADIF);
	OCR2 = ~(ADC/4) + 25;
	clear_display();
	sprintf(lcd_trigger, "%d",ADC);
	string2lcd(lcd_trigger);*/
	

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
		if(radio_state == 2){
			radio_pwr_dwn();
			radio_state = 0;
		}
		if(secs%2){
			segment_data[0] = BLANK;
			segment_data[1] = BLANK;
			segment_data[2] = BLANK;
			segment_data[3] = BLANK;
			segment_data[4] = BLANK;
		}
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
	if(radio_state == 1){
		radio_start();
		_delay_ms(10);
		radio_start();
		radio_state=2;
	}
	if(radio_state == 3){
		OCR3A = idle_volume;
		radio_pwr_dwn();
		radio_state=0;
	}
	if(!(current_fm_freq == prev_fm_freq)){
		fm_tune_freq();
		prev_fm_freq = current_fm_freq;
	}

	//*****display count*****//
	if(count > 1023){count = 0;}						//keep in range 0-1023
	if(count < 0){count = 1023;}						//keep in range 0-1023
	//segsum(count);								    //load display array
	//segsumTime();                                       //load display array with time
    if(digitCount == 6){digitCount= 0;}					//reset digit count to zero
	DDRA = 0xFF;										//PORTA output mode
    _delay_us(5);
	PORTA = dec_to_7seg[segment_data[digitCount]];		//send 7 segment code to LED segments
	PORTB = (0 << PWM) | (digitCount << 4);				//send PORTB the digit to display
	digitCount++;										//update digit to display

  }//while
}//main