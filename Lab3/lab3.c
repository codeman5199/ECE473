// lab3.c 

//Cody McCall

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 		1
#define FALSE 		0
#define BLANK		10			//blank digit is represented by the value ten
#define PWM			7			//pwm pin on PORTB
#define BTTN_EN		7			//decoder value to enable button tristate buffer
#define DISABLE		5			//unused decoder value to disable everything
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

uint8_t readFlag = 0;

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

//******************************************************************************
//							write_bargraph
//Sends value to bargraph via SPI. Pass in data value to be written to bargraph.
//Note MSB corresponds to bottom of bargraph.
//
void write_bargraph(uint8_t data){
	SPDR = data;
	while(bit_is_clear(SPSR,SPIF)) {}
	PORTB = 1;
	PORTB = 0;
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
	static uint16_t state = 0; //holds present state
	state = (state << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	if (state == 0xF000) return 1;
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

int readButton(button){
	//make PORTA an input port with pullups 
	PORTA = 0xFF;				//turn off display and prep for pullup values
	asm volatile("nop");
	asm volatile("nop");
	DDRA = 0x00;
  	//enable tristate buffer for pushbutton switches
	PORTB = (1 << PWM) | (BTTN_EN << 4);
  	//now check button 1 and change mode if needed
	for(uint8_t delay = 0; delay < 13; delay++){
		if(chk_buttons(button)){
			return TRUE;
		}
	}
	return FALSE;
}

//***********************************************************************************
uint8_t main()
{
DDRB = 0xF0;
static int count = 1;
uint8_t digitCount = 0b0;
uint8_t bttnCount = 0;
uint8_t press = FALSE;
while(1){

	if(readFlag == 1 | readFlag == 3){
		//quadread 1+2
	}
	else if(readFlag == 2){
		if(readButton(1)){

		}
	}
	else if(readFlag == 4){
		if(readButton(2)){

		}
	}
	
	if(count > 1023 | count < 0){count = 0;}		//keep in range 0-1023
	segsum(count);									//load display array
	if(digitCount == 5){digitCount= 0;}				//reset digit count to zero
	DDRA = 0xFF;									//PORTA output mode
	PORTA = dec_to_7seg[segment_data[digitCount]];	//send 7 segment code to LED segments
	PORTB = (0 << PWM) | (digitCount << 4);			//send PORTB the digit to display
	digitCount++;									//update digit to display
	//store count in display array
	//limit display count by digits
	//make PORTA output
	//send values to display
	//send PORTB digit to display
	//update display digit

	/*
  //insert loop delay for debounce
  //make PORTA an input port with pullups 
	PORTA = 0xFF;				//turn off display and prep for pullup values
	asm volatile("nop");
	asm volatile("nop");
	DDRA = 0x00;
  //enable tristate buffer for pushbutton switches
	PORTB = (1 << PWM) | (BTTN_EN << 4);
  //now check each button and increment the count as needed
	for(uint8_t delay = 0; delay < 13; delay++){
		if(chk_buttons(bttnCount)){
			press = TRUE;
		}
	}
	if(press == TRUE){
		//do thing depending on bttnCount
		count++;
	}
	press = FALSE;
  //disable tristate buffer for pushbutton switches
	PORTB = (1 << PWM) | (DISABLE << 4);
  //bound the count to 0 - 1023
	if((count == 1023) | (count == 0)){
		asm volatile("nop");
	}
  //break up the disp_value to 4, BCD digits in the array: call (segsum)
	segsum(count);
  //bound a counter (0-4) to keep track of digit to display 
	if(digitCount == 5){digitCount= 0;}				//reset digit count to zero
	if(bttnCount == 7){bttnCount= 0;}				//reset digit count to zero
  //make PORTA an output
	DDRA = 0xFF;
  //send 7 segment code to LED segments
	PORTA = dec_to_7seg[segment_data[digitCount]];	
  //send PORTB the digit to display
	PORTB = (0 << PWM) | (digitCount << 4);
  //update digit to display
	digitCount++;
	bttnCount++;
	*/
  }//while
}//main
