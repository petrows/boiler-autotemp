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

#include "lcd_lib.h"
#include "encoder.h"

void lcd_print(const char * str)
{
	while (0x00 != *str) { LCDsendChar(*str); str++; }
}

// Servo values (abs)
// MIN pos = 97
// MAX pos = 535
// MID pos = 316

#define SERVO_BASE 97 // Base freq param (register value = 97 + pos)
#define SERVO_MIN 28+SERVO_BASE
#define SERVO_MAX 128+SERVO_BASE

// Servi values (from 0)
// MIN pos = 0
// MAX pos = 438
// MID pos = 219

uint8_t servo_pers = 0;

void set_servo(uint8_t pers) // pos 0..100%
{
	// Real servo pos:
	// Min = 97 + 128
	// Max = 97 + 28
	
	if (pers > 100) pers = 100;
	
	servo_pers = pers;	
	uint16_t pers_set = 100 - pers;
	OCR1B = SERVO_MIN + pers_set;
}

void update_value(void)
{
	// Read new data
	ENC_PollEncoder();
	uint8_t enc_val = ENC_GetStateEncoder();
	if (0x00 == enc_val)
	{
		// Do nothing, noe wnew
	} else {
		if (RIGHT_SPIN == enc_val && servo_pers < 100)
		{
			set_servo(servo_pers+5);
		}
		if (LEFT_SPIN == enc_val && servo_pers > 0)
		{
			set_servo(servo_pers-5);
		}
	}
}

int main (void)
{
	//DDRB = 0xFF;
	//PORTB = 0x00;
	
	///ион - напряжение питания, выравнивание влево, нулевой канал
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	//вкл. ацп, режим постоянного преобр., разрешение прерывания,частота преобр. = FCPU/128
	ADCSRA = (1<<ADEN)|(0<<ADSC)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	
	//Configure TIMER1
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);		 	//NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);	//PRESCALER=64 MODE 14(FAST PWM)
	
	ICR1=4999;	//fPWM=50Hz (Period = 20ms Standard).
	
	DDRD = 0xFF; // Port in
	DDRD|=(1<<PD4)|(1<<PD5);	//PWM Pins as Out
	PORTD&=~(1<<PD4)|(1<<PD5);  //PWM pins value off
	
	set_servo(servo_pers);
	
	// return 0;
	_delay_ms(100);
	
	LCDinit();
	LCDclr();
	LCDGotoXY(0,0);
	lcd_print("SET    ");
	
	sei();
	
	// Start
	// ADCSRA |= (1<<ADSC);
	
	uint16_t servo_pos_old = 0;
	
	while (1)
	{
		//PORTB = ~PORTB;
		_delay_ms(1);
		
		update_value();
		
		if (servo_pers != servo_pos_old)
		{
			servo_pos_old = servo_pers;
			LCDGotoXY(0,1);
			char temp_c[4];
			if (servo_pos_old < 100)
			{
				LCDsendChar(' ');
				if (servo_pos_old < 10)
					LCDsendChar(' ');
			}
			itoa(servo_pos_old, temp_c, 10);
			lcd_print(temp_c);
		}
	}
	return 0;
}