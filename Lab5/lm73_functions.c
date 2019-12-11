// lm73_functions.c       
// Roger Traylor 11.28.10

#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>

//TODO: remove volatile type modifier?  I think so.
//TODO: initalize with more resolution and disable the smb bus timeout
//TODO: write functions to change resolution, alarm etc.

volatile uint8_t lm73_wr_buf[2];
volatile uint8_t lm73_rd_buf[2];

//********************************************************************************

void lm73_temp_init(){
    //set LM73 mode for reading temperature by loading pointer register
    lm73_wr_buf[0] = LM73_PTR_TEMP;               //load lm73_wr_buf[0] with temperature pointer address
    twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);   //start the TWI write process
    _delay_ms(2);                                 //wait for the xfer to finish
}

uint16_t lm73_temp_read(){
    uint16_t lm73_temp;
    twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);     //read temperature data from LM73 (2 bytes) 
    _delay_ms(2);                                   //wait for it to finish
    lm73_temp = lm73_rd_buf[0];                     //save high temperature byte into lm73_temp
    lm73_temp = lm73_temp << 8;                     //shift it into upper byte 
    lm73_temp |= lm73_rd_buf[1];                    //"OR" in the low temp byte to lm73_temp 
    //lm73_temp = lm73_temp >> 7;                     //drop decimals
    
    return lm73_temp;                               //return temperature value (converted)
}

//******************************************************************************
uint8_t lm73_temp_convert(char temp_digits[], uint16_t lm73_temp, uint8_t f_not_c){
//given a temperature reading from an LM73, the address of a buffer
//array, and a format (deg F or C) it formats the temperature into ascii in 
//the buffer pointed to by the arguement.
//TODO:Returns what???(uint8_t)??? Probably a BUG?

//Yeah, this is for you to do! ;^)
int temp = lm73_temp>>7;
if(f_not_c){
    sprintf(temp_digits, "%d F", (int)(((float)temp*1.8)+32));
}
else
{
    sprintf(temp_digits, "%d C", temp);
}


}//lm73_temp_convert
//******************************************************************************
