#include <avr/pgmspace.h>

static const int lcd_chars_count = 2;
static const PROGMEM unsigned char lcd_chars[] = {
	0b00111,
	0b00101,
	0b00111,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	
	0b00100,
	0b01110,
	0b11111,
	0b00000,
	0b11111,
	0b01110,
	0b00100,
	0b00000
	
};