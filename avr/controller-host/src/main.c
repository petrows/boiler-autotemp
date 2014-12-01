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

// =============================
// Global vars

uint8_t sensor_current; // Temperature sensor ADC value (current measured/rouded)
#define SENSOR_ADC_SIZE 4 // Size of measures list
uint8_t sensor_current_data[SENSOR_ADC_SIZE];

// =============================
// Protection limits

#define SERVO_LIMIT_MIN 125
#define SERVO_LIMIT_MAX 250

#define TEMP_SENSOR_MIN 20
#define TEMP_SENSOR_MAX 50

// =============================
// UART config

#define UART_BAUD 		9600L
#define UART_BAUD_DIV	(F_CPU/(16*UART_BAUD)-1)

// =============================
// ISR

ISR(ADC_vect) //подпрограмма обработки прерывания от АЦП
{
	// sensor_value = ADCH >> 2;
	// Calc avg sensor value
	uint8_t adc = ADCH;
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
	
	OCR1B = 125 + (sensor_current>>1);
}

// =============================
// Main

int main(void)
{
	// Init ports
	DDRD |= (1<<PD4); // Servo PWM at OC1B
	PORTD = 0x00; // LOW level on all pins port B
	
	// Set PWM TIMER1
	TCCR1A |= (1<<COM1B1)|(1<<WGM11);	// NON Inverted PWM 
	TCCR1B |= 	(1<<WGM13)|(1<<WGM12)	// Fast PWM
				| (1<<CS11)|(1<<CS10);	// Prescaler = 128

	ICR1  = 2499;		// fPWM=50Hz (Period = 20ms Standard).
	OCR1B = 125 + 0; 	// 0% pos
	
	// Restore last PWM value?
	
	for (uint8_t x=0; x<SENSOR_ADC_SIZE; x++)
	{
		sensor_current_data[x] = 0x00;
	}
	
	// Init ADC 
	// ион - напряжение питания, выравнивание влево, нулевой канал
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	// вкл. ацп, режим постоянного преобр., разрешение прерывания,частота преобр. = FCPU/128
	ADCSRA = (1<<ADEN)|(0<<ADSC)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	
	// Start
	ADCSRA |= (1<<ADSC);
	
	// Init LCD
	LCDinit();
	
	// Init almost done
	// Display welcome
	
	// Start ISR
	sei();
	
	while (1) {}
	return 0;
}
