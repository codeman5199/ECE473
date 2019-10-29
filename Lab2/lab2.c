// lab2.c 
// Cody McCall
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU	 16000000 			// cpu speed in hertz 
#define TRUE 			1
#define FALSE 			0
#define BLANK			10			//blank digit is represented by the value ten
#define PWM				7			//pwm pin on PORTB
#define BTTN_EN			7			//decoder value to enable button tristate buffer
#define DISABLE			5			//unused decoder value to disable everything
#define LED_ON_TIME		20			//number of times nop is executed at start of loop. Determines LED on time.
#define BTTN_ON_TIME	15			//number of times single button is polled in loop
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] ={
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
0b11111111,		//10 or blank
};


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
	static uint16_t state[8] = {0,0,0,0,0,0,0,0}; //holds present state, each button has it's own state
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;			//shift in state of button pin to appropriate state variable
	if (state[button] == 0xF000) return 1;
	return 0;
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
	uint8_t numDigits= 0;
	uint8_t ones,tens,hundreds,thousands;
	if(sum == 0){								//special case for count = 0
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
	tens = (((sum%1000)%100)/10);						//tens place value, mod removes hundreds and thousands place
	ones = (((sum%1000)%100)%10);							//ones place value, modding out everything but the ones place
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
//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
DDRB = 0xF0;
static int count = 0;					//overall count to display
uint8_t digitCount = 0b0;				//tracks current digit
uint8_t bttnCount = 0;					//tracks current button
while(1){
	segsum(count);												//break count into 4 digits and populate segment_data[] array
  //loop delay for debounce
	for(uint8_t LEDDelay = 0; LEDDelay < LED_ON_TIME; LEDDelay++){ 		//loop nop for set number. This controls how long the LEDs are on for
		asm volatile("nop");
	}
  //initialize PORTA as input port with pullups 
	PORTA = 0xFF;
	DDRA = 0x00;
	asm volatile("nop");										//delay to allow pinchange to update
	asm volatile("nop");
  //enable tristate buffer for pushbutton switches
	PORTB = (BTTN_EN << 4);										//PORTB = 0b01110000
  //check current button (according to bttnCount)	
	for(uint8_t delay = 0; delay < BTTN_ON_TIME; delay++){				//loop for 15 cycles (minimum to catch in input is 12
		if(chk_buttons(bttnCount)){								//if button is pressed, only triggers once per press
			switch(bttnCount){
				case 0:
					count++;									//add 1
					break;
				case 1:
					count = count + 2; 							//add 2
					break;
				case 2:
					count = count + 4;							//add 4
					break;
				case 3:
					count = count + 8;							//add 8
					break;
				case 4:
					count = count + 16;							//add 16
					break;
				case 5:
					count = count + 32;							//add 32
					break;
				case 6:
					count = count + 64;							//add 64
					break;
				case 7:
					count = count + 128; 						//add 128
					break;
			}
		}
	}
	bttnCount++;
  //disable tristate buffer for pushbutton switches
	PORTB = (1 << PWM) | (DISABLE << 4);
  //bound the count to 0 - 1023
	if (count > 1023) count= count - 1023;
  //bound a counter (0-4) to keep track of digit to display 
	if(digitCount == 5){digitCount= 0;}				//reset digit count to zero
	if(bttnCount == 8){bttnCount= 0;}				//reset digit count to zero
  //make PORTA an output
	DDRA = 0xFF;
  //send 7 segment code to LED segments
	PORTA = dec_to_7seg[segment_data[digitCount]];	
  //send PORTB the digit to display
	PORTB = (0 << PWM) | (digitCount << 4);
  //update digit to display
	digitCount++;
  }//while
}//main
