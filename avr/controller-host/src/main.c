#ifndef F_CPU
#	define F_CPU 8000000
#endif

#ifndef __AVR_ATmega16__
#	define __AVR_ATmega16__
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 
#include <avr/cpufunc.h>

#include <avr/iom16.h>

#include <stdlib.h>

#include "common.h"
#include "lcd_char.h"
#include "lcd_lib.h"
#include "encoder.h"

int main(void)
{
	while (1) {}
	return 0;
}
