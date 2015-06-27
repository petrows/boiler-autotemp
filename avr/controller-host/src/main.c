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

void encoderUpdate(void);
void servoUpdate(void);

// =============================
// Global vars

uint8_t sensor_current = 0x00; // Temperature sensor ADC value (current measured/rouded)
uint8_t sensor_current_prev = 0x00;
#define SENSOR_ADC_SIZE 4 // Size of measures list
uint8_t sensor_current_data[SENSOR_ADC_SIZE];

uint8_t encoder_current = 40;

uint16_t seconds_count = 0x00;
uint8_t seconds_time_sec = 0x00;
uint8_t seconds_time_min = 0x00;

uint8_t servo_current = 0x00;

float char_persent = 100.0 / 255.0;

uint8_t display_update_flag = 0x00;

float Kp = 5;
float Ki = 0.2;

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
// Strings

const uint8_t str_welcome[] PROGMEM = "BOLIER  INIT\0";
const uint8_t str_no_value_2[] PROGMEM = "--\0";

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
		sensor_current = 0xFF - adc;
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
		
		sensor_current = 0xFF - adc_summ;
	}
	
	if (sensor_current_prev != sensor_current)
	{
		display_update_flag = 1;
		sensor_current_prev = sensor_current;
	}
}

uint16_t timer0_counter_second = 0x0000;
uint8_t timer0_counter_led_second = 0x00;

// Timer0 vector function
// 1 ms freq
ISR(TIMER0_OVF_vect)
{
	timer0_counter_second++;	
	if (1000 == timer0_counter_second) // 1 second counter
	{
		// Every second
		timer0_counter_second = 0;
		seconds_count++;
		seconds_time_sec++;
		if (60 == seconds_time_sec)
		{
			seconds_time_min++;
			seconds_time_sec = 0;
		}
		
		display_update_flag = 1;
		
		PORTD |= (1<<PD5);
		
		servoUpdate();
	}
	
	encoderUpdate();
	
	TCNT0 = 125;
}

// =============================
// Common

void servoUpdate(void)
{
	servo_current = sensor_current;
	OCR1B = 125 + ((0xFF - servo_current)>>1);
}

void encoderUpdate(void)
{
	// Read new data
	ENC_PollEncoder();
	uint8_t enc_val = ENC_GetStateEncoder();
	if (0x00 == enc_val)
	{
		// Do nothing, noe wnew
	} else {
		if (RIGHT_SPIN == enc_val)
		{
			if (encoder_current < 255)
				encoder_current++;
		}
		if (LEFT_SPIN == enc_val)
		{
			if (encoder_current > 0)
				encoder_current--;
		}
	}
}

void displayUpdate(void)
{
	uint8_t display_servo_current = servo_current;
	//uint8_t display_servo_current = char_persent * (float)servo_current;
	char temp_c[8];
	itoa((uint8_t)display_servo_current, temp_c, 10);
	LCDGotoXY(12,1);
	if (display_servo_current < 100) LCDsendChar(' ');
	if (display_servo_current < 10) LCDsendChar(' ');
	lcdPrint(temp_c);
	
	uint8_t display_seconds_time_sec = seconds_time_sec;
	uint8_t display_seconds_time_min= seconds_time_min;
	LCDGotoXY(10,0);
	if (display_seconds_time_min < 100) LCDsendChar(' ');
	if (display_seconds_time_min < 10) LCDsendChar(' ');
	itoa(display_seconds_time_min, temp_c, 10); lcdPrint(temp_c);
	LCDsendChar(':');
	LCDGotoXY(14,0);
	if (display_seconds_time_sec < 10) LCDsendChar('0');
	itoa(display_seconds_time_sec, temp_c, 10); lcdPrint(temp_c);
	
	uint8_t display_encoder_current = encoder_current;
	itoa(display_encoder_current, temp_c, 10);
	LCDGotoXY(3,0);
	if (display_encoder_current < 10) LCDsendChar(' ');
	lcdPrint(temp_c);
	
	uint8_t display_sensor_current = 25 + (sensor_current/10);
	itoa(display_sensor_current, temp_c, 10);
	LCDGotoXY(3,1);
	if (display_sensor_current < 10) LCDsendChar(' ');
	lcdPrint(temp_c);
}

void displayModeTemp(void)
{
	LCDclr();
	
	LCDGotoXY(0,0);
	//lcd_print("SET    ");
	LCDsendChar(0x01);
	LCDGotoXY(5,0); LCDsendChar(0x00); LCDsendChar('C');
	//LCDGotoXY(6,0); LCDsendChar('C');// LCDsendChar('C');
	LCDGotoXY(0,1);
	LCDsendChar('t');
	LCDGotoXY(5,1); LCDsendChar(0x00); LCDsendChar('C');
	
	LCDGotoXY(15,1); LCDsendChar('"');
	
	CopyStringtoLCD(str_no_value_2, 3, 0);
	CopyStringtoLCD(str_no_value_2, 3, 1);
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
	
	// Enable timer vect
	TIMSK = (1<<TOIE0);
	
	// Set common TIMER0
	TCCR0 |= (0<<CS02) | (1<<CS01) | (1<<CS00);	// Prescaler 64
	// set timer0 counter initial value to 125
	TCNT0 = 125;

	// Set PWM TIMER1
	TCCR1A |= (1<<COM1B1)|(1<<WGM11);	// NON Inverted PWM 
	TCCR1B |= 	(1<<WGM13)|(1<<WGM12)	// Phase PWM
				| (1<<CS11)|(1<<CS10);	// Prescaler = 128

	ICR1  = 2499;		// fPWM=50Hz (Period = 20ms Standard).
	OCR1B = 125 + 0; 	// 0% pos
	
	// Restore last PWM value?
	
	// reset timer counter
	SFIOR |= (1<<PSR10);
	
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
	
	// Encoder
	ENC_InitEncoder();
	
	// Start ISR
	sei();
	
	_delay_ms(100);
	
	// Init LCD
	LCDinit();
	_delay_ms(100);
	// Install custom chars
	for(int i=0; i<lcd_chars_count; i++)
	{
		LCDdefinechar(lcd_chars + (i << 3), i);			
	}
	
	// Init almost done
	displayModeTemp();
	// Display welcome
	// CopyStringtoLCD(str_welcome, 2, 0);
	
	while (1) 
	{
		displayUpdate();
		while (0x00 == display_update_flag) {};
	}
	return 0;
}
