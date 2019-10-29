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
#include <avr/interrupt.h>
#include <util/delay.h>

static uint8_t readFlag = 0;

ISR(TIMER0_OVF_vect){
	readFlag++;								//increment readFlag on interrupt
	if(readFlag == 3){readFlag = 0;}		//reset when get to 5
}

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

static int modeSum = 1;


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
//                                   readButton
//read selected button for full debounce cycle. Button selected by changing button param
//
int readButton(button){
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

void setModeSum(int mode){
	switch(mode){
		case 0x00:
			modeSum = 1;
			break;
		case 0x02:
			modeSum = 2;
			break;
		case 0x04:
			modeSum = 4;
			break;
		case 0x06:
			modeSum = 0;
			break;
	}
}

//***********************************************************************************
//                                   quad1Read
//read left encoder and return pulse for each cw or ccw tick on encoder. Populate quad1_prev
//and quad1_current before calling funciton. Returns 1 for ccw, 2 for cw and 0 for no turn.
//
int quad1Read(uint8_t quad1_prev, uint8_t quad1_current){
	static int8_t count1;										//counter for quadrature edges
	if(quad1_current == quad1_prev){							//no turn case
		return 0;
	}
	switch(quad1_prev){											//compare previous measured quad value with current one
		case 0b00:
			if(quad1_current == 0b10){count1++;}				//ccw
			else if(quad1_current == 0b01){count1--;}			//cw
			break;
		case 0b01:
			if(quad1_current == 0b00){count1++;}				//ccw
			else if(quad1_current == 0b11){count1--;}			//cw
			break;
		case 0b10:
			if(quad1_current == 0b11){count1++;}				//ccw
			else if(quad1_current == 0b00){count1--;}			//cw
			break;
		case 0b11:
			if(quad1_current == 0b01){count1++;}				//ccw
			else if(quad1_current == 0b10){count1--;}			//cw
			break;
	}	
	if(count1 == 4){											//count 4 edges for one tick
		count1 = 0;												//reset counter
		return 1;												//return 1 for ccw
	}
	else if(count1 == -4){										//count 4 edges for one tick
		count1 = 0;												//reset counter
		return 2;												//return 2 for cw
	}
}

int quad2Read(uint8_t quad2_prev, uint8_t quad2_current){
	static int8_t count2;
	if(quad2_current == quad2_prev){
		return 0;
	}
	switch(quad2_prev){								//compare previous measured quad value with current one
		case 0b00:
			if(quad2_current == 0b10){count2++;}				//ccw
			else if(quad2_current == 0b01){count2--;}			//cw
			break;
		case 0b01:
			if(quad2_current == 0b00){count2++;}				//ccw
			else if(quad2_current == 0b11){count2--;}			//cw
			break;
		case 0b10:
			if(quad2_current == 0b11){count2++;}				//ccw
			else if(quad2_current == 0b00){count2--;}			//cw
			break;
		case 0b11:
			if(quad2_current == 0b01){count2++;}				//ccw
			else if(quad2_current == 0b10){count2--;}			//cw
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
}

//***********************************************************************************
uint8_t main()
{
DDRB = 0xFF;
DDRE = 0xC0;
PORTE = 0b11000000;
TIMSK |= (1<<TOIE0);             //enable interrupts
TCCR0 |= (1<<CS02) | (1<<CS00);  //normal mode, prescale by 128
uint8_t quad1_prev = 0;
uint8_t quad1_current = 0;
uint8_t quad2_prev = 0;
uint8_t quad2_current = 0;
int mode = 0;
int turn = 0;
int data = 0;
static int count = 1;
uint8_t digitCount = 0;
spi_init();
sei();
while(1){

	//if((readFlag == 1) | (readFlag == 3)){
	if((readFlag == 1) ){
		//quadread 1+2
		PORTE = 0b01000000;							//toggle SDLD and CLK_INH
		SPDR = 0x00;								//write garbage
		while(bit_is_clear(SPSR, SPIF)){}			//spin until complete
		quad1_prev = quad1_current;					//set previous value to current
		quad2_prev = quad2_current;					//set previous value to current
		data = SPDR;								//read new current value
		quad1_current = (data & 0x03);				//parse new value
		quad2_current = (data & 0x0C)>>2;				//parse new value
		turn = quad1Read(quad1_prev, quad1_current);//eval state machine
		if(turn == 1){count = count + modeSum;}
		if(turn == 2){count = count - modeSum;}
		turn = quad2Read(quad2_prev, quad2_current);//eval state machine
		if(turn == 1){count = count + modeSum;}
		if(turn == 2){count = count - modeSum;}
		//if(turn == 1){write_bargraph(1);}
		//if(turn == 2){write_bargraph(2);}
		PORTE = 0b10000000;
	}
	else if(readFlag == 2){
		if(readButton(0)){
			mode ^= 0x02;
		//	setModeSum(mode, modeSum);
		}
	//}
	//else if(readFlag == 4){
		else if(readButton(1)){
			mode ^= 0x04;
		}
		write_bargraph(mode);	
		setModeSum(mode);
	}
	
	if(count > 1023){count = 0;}					//keep in range 0-1023
	if(count < 0){count = 1023;}					//keep in range 0-1023
	segsum(count);									//load display array
	if(digitCount == 6){digitCount= 0;}				//reset digit count to zero
	DDRA = 0xFF;									//PORTA output mode
	PORTA = dec_to_7seg[segment_data[digitCount]];	//send 7 segment code to LED segments
	PORTB = (0 << PWM) | (digitCount << 4);			//send PORTB the digit to display
	digitCount++;									//update digit to display

  }//while
}//main
