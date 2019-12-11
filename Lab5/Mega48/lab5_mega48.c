#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lm73_functions.h"
#define TEMP_CMD  'r'

char command;
uint8_t getTemp = 0;
uint16_t tempval;
char tx_buf_to128[16];

int verify_command(char data){
    if(data == TEMP_CMD){return 1;}
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
            tempval = lm73_temp_read();
            lm73_temp_convert(tx_buf_to128, tempval, 0);
            getTemp = 0;
            //report to 128
            uart_puts(tx_buf_to128);
        }
    }
}