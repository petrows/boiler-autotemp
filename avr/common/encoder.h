#ifndef	encoder_h
#define	encoder_h
#include <avr/io.h>
//_________________________________________
//порт и выводы к которым подключен энкодер
#define PORT_Enc 	PORTC 	
#define PIN_Enc 	PINC
#define DDR_Enc 	DDRC
#define Pin1_Enc 	0
#define Pin2_Enc 	1
//______________________
#define RIGHT_SPIN 0x01 
#define LEFT_SPIN 0xff

void ENC_InitEncoder(void);
void ENC_PollEncoder(void);
unsigned char ENC_GetStateEncoder(void);
#endif  //encoder_h
