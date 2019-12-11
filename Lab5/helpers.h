#define HELPERS_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "globals.h"
void spi_init();
void write_bargraph(uint8_t);
uint8_t chk_buttons(uint8_t);
void segsum(uint16_t);
void updateTime();
void segsumTime();
void segsumAlarm();
int readButton(uint8_t);
int quad1Read(uint8_t, uint8_t);
int quad2Read(uint8_t, uint8_t);
void incSetTime();
void decSetTime();
void incdec_vol(uint8_t);
void readInputs();
void initTimers();
