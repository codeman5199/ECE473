#define TEMP_CMD	'r'
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "uart_functions.h"

uint8_t main(){
    uart_init();						//initalize uart with interrupts
    while(1){
        uart_putc('r');
        _delay_ms(1000);
    }
}