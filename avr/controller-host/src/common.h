#ifndef _COMMON_H
#define _COMMON_H

#include "lcd_lib.h"

#define HIBYTE(a) 	(*((uint8_t*)&(a) + 1))
#define LOBYTE(a) 	(*((uint8_t*)&(a)))
#define HIWORD(a) 	(*((uint16_t*)&(a) + 1))
#define LOWORD(a) 	(*((uint16_t*)&(a)))
#define HI(x) 		((x)>>8)
#define LO(x) 		((x)& 0xFF) 

void lcdPrint(const char * str)
{
	while (0x00 != *str) { LCDsendChar(*str); str++; }
}

void uart_send_byte(char byte)
{
	while(!(UCSRA & (1<<UDRE)));
	UDR = byte;
}

void uart_send_string(char * str)
{
	while (*str)
	{
		uart_send_byte(*str);
		str++;
	}
}

#endif