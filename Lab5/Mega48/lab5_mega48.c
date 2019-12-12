#define F_CPU 16000000 // cpu speed in hertz 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lm73_functions.h"
#include "uart_functions_m48.h"
#define TEMP_CMD  'r'

volatile char command;
volatile uint8_t getTemp;
uint16_t tempval;
char tx_buf_to128[16] = {'6' , '9', 'C', '\0'};

int verify_command(char data){
    if(data == TEMP_CMD){return 1;}
    else {return 0;}
}

ISR(USART_RX_vect){
    getTemp = 1;
    command = (char)UDR0;
    // if(command == 'r'){
    //     getTemp = 1;
    // }
    // if(command == 'r'){
    //     getTemp = 1;
    // }
    // else
    // {
    //     //respond error
    // }
}

uint8_t main(){
    DDRD = (1<<PD4);
    uart_init();
    init_twi();
    sei();
    //uart_putc('s');
    uint8_t Temp;
    while(1){
        //uart_putc('t');
        //getTemp = 1;
        //_delay_ms(500);
        // if(uart_getc() == 'TEMPCMD'){
        //     getTemp = 1;
        // }
        if((getTemp == 1)){
            tempval = lm73_temp_read();
            lm73_temp_convert(tx_buf_to128, tempval, 0);
            //report to 128
            uart_putc(' ');
            uart_puts(tx_buf_to128);
            getTemp = 0;
        }
    }
}