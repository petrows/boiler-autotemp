#include "encoder.h"

#define SetBit(port, bit) port|= (1<<bit)
#define ClearBit(port, bit) port&= ~(1<<bit)

//это для наглядности кода
#define b00000011 3
#define b11010010 210
#define b11100001 225

volatile unsigned char bufEnc = 0; //буфер энкодера

//функция инициализации
//__________________________________________
void ENC_InitEncoder(void)
{
	ClearBit(DDR_Enc, Pin1_Enc); //вход
	ClearBit(DDR_Enc, Pin2_Enc);
	SetBit(PORT_Enc, Pin1_Enc);//вкл подтягивающий резистор
	SetBit(PORT_Enc, Pin2_Enc);
}

//функция опроса энкодера
//___________________________________________
void ENC_PollEncoder(void)
{
	static unsigned char stateEnc; 	//хранит последовательность состояний энкодера
	unsigned char tmp;  
	unsigned char currentState = 0;
	
	//проверяем состояние выводов микроконтроллера
	if ((PIN_Enc & (1<<Pin1_Enc))!= 0) {SetBit(currentState,0);}
	if ((PIN_Enc & (1<<Pin2_Enc))!= 0) {SetBit(currentState,1);}
	
	//если равно предыдущему, то выходим
	tmp = stateEnc;
	if (currentState == (tmp & 0b00000011)) return;
	
	//если не равно, то сдвигаем и сохраняем в озу
	tmp = (tmp<<2)|currentState;
	stateEnc = tmp;
	
	//сравниваем получившуюся последовательность
	if (tmp == 0b11100001) bufEnc = LEFT_SPIN;
	if (tmp == 0b11010010) bufEnc = RIGHT_SPIN;
	return;
}

//функция возвращающая значение буфера энкодера
//_____________________________________________
unsigned char ENC_GetStateEncoder(void)
{
	unsigned char tmp = bufEnc;
	bufEnc = 0;
	return tmp;
}


