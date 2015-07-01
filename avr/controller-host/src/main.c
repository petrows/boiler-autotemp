#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 
#include <avr/cpufunc.h>

#include <stdlib.h>

#include "common.h"
#include "lcd_char.h"
#include "lcd_lib.h"
#include "encoder.h"

void encoderUpdate(void);
void controlUpdate(void);
void servoSet(uint8_t pers);
void displayError(uint8_t code);
uint8_t sensorToTemp(uint8_t value); // Values - t(c)
void uartSend(void);

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

volatile uint8_t display_update_flag = 0x00;

float Kp = 5;
float Ki = 0.2;

// =============================
// Protection limits

#define SERVO_BASE 97 // Base freq param (register value = 97 + pos)
#define SERVO_MIN 28+SERVO_BASE
#define SERVO_MAX 128+SERVO_BASE

#define TEMP_SENSOR_MIN 20
#define TEMP_SENSOR_MAX 50

// =============================
// UART config

#define UART_BAUD 		9600L
#define UART_BAUD_DIV	(F_CPU/(16*UART_BAUD)-1)

uint8_t uart_data[3];

// =============================
// Common

#define ERROR_NO_SENSOR 0x01

// =============================
// Strings

const uint8_t str_welcome[] PROGMEM = "BOLIER  INIT\0";
const uint8_t str_no_value_2[] PROGMEM = "--\0";
const uint8_t str_error[] PROGMEM = "ERROR\0";

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
	
	sensor_current = sensorToTemp(sensor_current);
	
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
		
		// PORTD |= (1<<PD5);
		
		controlUpdate();
	}
	
	encoderUpdate();
	
	TCNT0 = 125;
}

// =============================
// Common

uint8_t sensorToTemp(uint8_t value)
{
	// Max value is 0xFF (temp 25)
	// Min value is 0x00 (temp is 50)
	
	if (0xFF == value)
	{
		return 0; // This is an error...
	}
	
	uint8_t temp40cal = 165; // Value of 40C (calibration)
	float tempCoef = 8.7; // Values/C size
	float tempDiff = temp40cal-value;
	
	if (value <= temp40cal)
	{
		return 40 + ((float)(tempDiff)/tempCoef);
	} else {
		return 40 - ((float)(-tempDiff)/tempCoef);
	}
}

void controlUpdate(void)
{
	// Error?
	if (sensor_current < 20)
	{
		// Sensor ERROR
		displayError(ERROR_NO_SENSOR);
	}
	
	if (sensor_current != encoder_current)
	{
		if (sensor_current < encoder_current)
		{
			if (servo_current < 100) servoSet(servo_current + 1);
		} else {
			if (servo_current > 0) servoSet(servo_current - 1);
		}
	}
	
	// Send current value
	uart_data[2] = sensor_current;
	uart_data[1] = servo_current;
	uart_data[0] = 0xFF; // Enable send!
}

void servoSet(uint8_t pers)
{
	if (pers > 100) pers = 100;
	
	servo_current = pers;	
	uint16_t pers_set = 100 - pers;
	OCR1B = SERVO_MIN + pers_set;
}

void uartSend(void)
{
	for (int x=0; x<3; x++)
	{
		while(!(UCSRA & (1<<UDRE)));
		UDR=uart_data[x];
		uart_data[x] = 0x00;
	}
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

void displayError(uint8_t code)
{
	LCDclr();
	LCDGotoXY(0,0);
	CopyStringtoLCD(str_error, 2, 0);
	LCDsendChar(' ');
	char temp_c[8];
	itoa(code, temp_c, 10);
	lcdPrint(temp_c);
	
	// Zero throttle
	servoSet(0);
	
	display_update_flag = 0x01;
	cli(); // Disable interrupts! == halt process
	exit(0);
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
	
	uint8_t display_sensor_current = sensor_current;
	itoa(display_sensor_current, temp_c, 10);
	LCDGotoXY(3,1);
	if (display_sensor_current < 10) LCDsendChar(' ');
	lcdPrint(temp_c);
	
	display_update_flag = 0x00;
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
	
	LCDGotoXY(15,1); LCDsendChar('%');
	
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
	ADCSRA = (1<<ADEN)|(0<<ADSC)|(1<<ADFR)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	
	// Start
	ADCSRA |= (1<<ADSC);
	
	// Configure UART
	UBRRL = LO(UART_BAUD_DIV);
	UBRRH = HI(UART_BAUD_DIV);
	UCSRA = 0;
	UCSRB = 1<<RXEN|1<<TXEN|0<<RXCIE|0<<TXCIE;
	UCSRC = 1<<URSEL|1<<UCSZ0|1<<UCSZ1;
		
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
	
	for (int x=0; x<3; x++) uart_data[x] = 0x00;
	
	// Init almost done
	displayModeTemp();
	// Display welcome
	// CopyStringtoLCD(str_welcome, 2, 0);
	
	while (1) 
	{
		_delay_ms(1);
		if (0x00 != display_update_flag)
		{
			displayUpdate();
		}
		
		if (uart_data[0] != 0x00)
		{
			uartSend();
		}
	}
	return 0;
}
