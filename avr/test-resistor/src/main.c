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

// =============================
// Global vars

volatile uint8_t sensor_current = 0x00; // Temperature sensor ADC value (current measured/rouded)
volatile uint8_t sensor_current_prev = 0x00;
#define SENSOR_ADC_SIZE 4 // Size of measures list
volatile uint8_t sensor_current_data[SENSOR_ADC_SIZE];

// =============================
// ISR

// ADC vector function
ISR(ADC_vect)
{
	// Calc avg sensor value
	uint8_t adc = ADCH;
	if (0x00 == sensor_current)
	{
		// First run, fill the array
		for (int x=0; x<SENSOR_ADC_SIZE; x++)
		{
			sensor_current_data[x] = adc;
		}
		sensor_current = adc;
	} else {
		uint16_t adc_summ = 0x00;
		for (int x=1; x<SENSOR_ADC_SIZE; x++)
		{
			sensor_current_data[x] = sensor_current_data[x-1];
			adc_summ = adc_summ + sensor_current_data[x];
		}
		sensor_current_data[0] = adc;
		adc_summ = adc_summ + adc;
		adc_summ = adc_summ>>2; // adc_summ/4
		
		sensor_current = adc_summ;
	}
	
	// sensor_current = sensor_current >> 1;
}

// =============================
// Main

int main(void)
{
	// Init ports
	DDRD = 0xFF;
	DDRD |= (1<<PD4); // Servo PWM at OC1B
	DDRD |= (1<<PD5); // Led at PD5
	PORTD = 0x00; // LOW level on all pins port B
	
	DDRB  = 0xFF;
	PORTB = 0x00;
	
	DDRC = 0x00; // C port IN
		
	// ADC init
	for (uint8_t x=0; x<SENSOR_ADC_SIZE; x++)
	{
		sensor_current_data[x] = 0x00;
	}
	// ион - напряжение питания, выравнивание влево, нулевой канал
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	// вкл. ацп, режим постоянного преобр., разрешение прерывания,частота преобр. = FCPU/128
	ADCSRA = (1<<ADEN)|(0<<ADSC)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	
	// Start
	ADCSRA |= (1<<ADSC);
	
	// Start ISR
	sei();
	
	_delay_ms(100);
	
	// Init LCD
	LCDinit();
	_delay_ms(100);
	
	LCDclr();
	
	LCDGotoXY(0,0);
	lcdPrint("Test prog");
	
	while (1) 
	{
		if (sensor_current_prev != sensor_current)
		{
			uint8_t temp_print = sensor_current;
			//LCDGotoXY(0,1);
			//lcdPrint("   ");
			LCDGotoXY(0,1);
			char temp_c[8];
			itoa((uint8_t)temp_print, temp_c, 10);
			if (temp_print<10) LCDsendChar(' ');
			if (temp_print<100) LCDsendChar(' ');
			lcdPrint(temp_c);
			sensor_current_prev = sensor_current;
			
			_delay_ms(100);
		}
	}
	return 0;
}
