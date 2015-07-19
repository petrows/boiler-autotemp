#ifndef _COMMON_H
#define _COMMON_H

#include <avr/sfr_defs.h>

#include "lcd_lib.h"

#define HIBYTE(a) 	(*((uint8_t*)&(a) + 1))
#define LOBYTE(a) 	(*((uint8_t*)&(a)))
#define HIWORD(a) 	(*((uint16_t*)&(a) + 1))
#define LOWORD(a) 	(*((uint16_t*)&(a)))
#define HI(x) 		((x)>>8)
#define LO(x) 		((x)& 0xFF) 

#define BIT1(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  // old sbi()
#define BIT0(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // old cbi()
#define BITX(sfr, bit) (_SFR_BYTE(sfr) ^= _BV(bit))  

#define MAX(x, y)	(((x) > (y)) ? (x) : (y))
#define MIN(x, y)	(((x) < (y)) ? (x) : (y))

void lcdPrint(const char * str)
{
	while (0x00 != *str) { LCDsendChar(*str); str++; }
}

void uartSendByte(char byte)
{
	while(!(UCSRA & (1<<UDRE)));
	UDR = byte;
}

void uartSendString(char * str)
{
	while (*str)
	{
		uartSendByte(*str);
		str++;
	}
}

#endif