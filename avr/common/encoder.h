#ifndef	encoder_h
#define	encoder_h
#include <avr/io.h>
//_________________________________________
//порт и выводы к которым подключен энкодер
#define PORT_Enc 	PORTD 	
#define PIN_Enc 	PIND
#define DDR_Enc 	DDRD
#define Pin1_Enc 	2
#define Pin2_Enc 	3
//______________________
#define RIGHT_SPIN 0x01 
#define LEFT_SPIN 0xff

void ENC_InitEncoder(void);
void ENC_PollEncoder(void);
unsigned char ENC_GetStateEncoder(void);
#endif  //encoder_h
