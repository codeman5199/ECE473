#ifndef GLOBALS_H
#define GLOBALS_H

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 		1
#define FALSE 		0
#define BLANK		10			//blank digit is represented by the value ten
#define COLON_ON		11			//colon on is represented by the value eleven
#define PWM			7			//pwm pin 7 on PORTB
#define BTTN_EN		7			//decoder pin cpnnected to enable of tristate buffer
#define DISABLE		5			//unused decoder value for disabling
#define SH_LD		6			//SH/LD pin 6 on PORTE
#define CLK_INH		7			//CLK_INH pin 7 on PORTE
#define SETMINS		0
#define	SETHOURS	1
#define DISP_TOGGLE	2
#define ARMALARM	3
#define SNOOZE		7

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


char lcd_armed[16] = {'A', 'R', 'M', 'E', 'D', ' '};  //holds string to send to lcd 
char lcd_beep[16] = {'B', 'E', 'E', 'P'};  //holds string to send to lcd  
char lcd_snooze[16] = {'S', 'N', 'O', 'O', 'Z', 'E'};  //holds string to send to lcd 
char lcd_trigger[16];

//******************************************************************************
//							button_read
//flag to control which button is being polled at any time in the ISR
//this is only to be edited within the TIMER0_OVF_vect ISR.
//
static uint8_t button_read = 0;	

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[14] ={
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
};

#endif