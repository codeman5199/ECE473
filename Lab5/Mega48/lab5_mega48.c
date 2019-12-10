#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lm73_functions.h"
#define RX_CMD  'r'

char command;
uint8_t getTemp = 0;

int verify_command(char data){
    if(data == RX_CMD){return 1;}
    else {return 0;}
}

ISR(USART_RX_VECT){
    command = uart_getc();
    if(verify_command(command)){
        //blink LED?
        getTemp = 1;
    }
    else
    {
        //respond error
    }
}

uint8_t main(){
    uart_init();
    init_twi();
    sei();
    uint8_t Temp;
    while(1){
        if(getTemp){
            twi_start_rd(TEMP_ADDR, &Temp, 1);          //read temp over TWI
            lm73_temp_convert(char temp_digits[], Temp, uint8_t f_not_c)
            getTemp = 0;
            //report to 128
        }
    }
}