#define F_CPU 16000000UL

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "adc.h"
#include "lidar.h"
#include "gyro.h"

uint8_t OPENINGS = 40;
uint8_t g_leftCount = 0;
uint8_t g_rightCount = 0;
bool g_readingDone = true;
bool g_sentData = false;
/*
 * TODO:
 * implement storage in memory where all data is stored
 * implement communication with other devices
 * move around functions to correct positions
 * testing functionality:
 * IR sensors
 * lidar sensor/s
 * odometer/s
 * MLX gyro
 */

void StartReading()
{
	g_readingDone = false;
	cli();
	AdcInit();
	sei();
	StartAdc();
	MeasureLidar();
	while(!g_IRDone){}
	StartMLX();
}

void PinInit()
{
	// define output pins
	DDRB =  (1 << PORTB4) | (1 << PORTB6);
	PORTB = (1 << PORTB4) | (1 << PORTB6);
	DDRA = (1 << PORTA4);
	PORTA = 0x00;
	DDRC = (1 << PORTC4);
	PORTC = 0x00;
	DDRD = (1 << PORTD0);
	PORTD = 0x00;
}
int main(void)
{
	PinInit();
	TimerInit();
	ExtInterruptInit();
	MsTimerInit();
	sei();
	StartReading();
    while (1) 
    {
		if(g_sentData)
		{
			StartReading();	
		}
    }
}

void ConvertOdo()
{
	// converts odo count to mm traveled;
	uint8_t res = 0;
	res = round(g_leftCount * 65 * M_PI / OPENINGS);
	g_leftCount = 0;
	// store res
	res = round(g_rightCount * 65 * M_PI / OPENINGS);
	g_rightCount = 0;
	// store res
}

ISR(ADC_vect)
{
	if (ADMUX & 0x46) // reading from MLX
	{
		cli();
		g_angle += MLXGyroVal();
		sei();
	}
	else // reading from IR
	{
		double ADCVoltage = 0;
		uint8_t IRDistance = 0;
		uint8_t ADCLowBit = ADCL;
		uint16_t ADCRes = ADCH<<8 | ADCLowBit; // puts result of ADC in ADCRes
		ADCVoltage = ADCRes * 5 / 1024;
		cli();
		IRDistance = ConvertVoltage(ADCVoltage);
		sei();
		// store value in correct place in memory
		NextInputPin(); //update ADMUX
		// update memory for next ad conversion
	}
}

ISR(PCINT1_vect)
{
	uint16_t PWMTime = 0;
	uint16_t lidarDistance = 0;
	if ((PORTB & 0x20) && !(PORTB & 0x10)) // if PB5 is high but not PB4
	{
		TimerInit();
		while (PORTB & 0x20){}
		TimerStop();
		cli();
		PWMTime = TCNT1;
		sei();
		lidarDistance = PWMTime / 2;
	}
	else if ((PORTB & 0x80) && !(PORTB & 0x40)) // if PB7 is high but not PB6 
	{
		TimerInit();
		while (PORTB & 0x80){}
		TimerStop();
		cli();
		PWMTime = TCNT1;
		sei();
		lidarDistance = PWMTime / 2;
	}
	// save lidar distance
}
ISR(PCINT2_vect)
{
	if(PORTC & 0x01)
	{
		g_leftCount++;
	}
	else
	{
		g_rightCount++;
	}
}
ISR(TIMER3_COMPA_vect)
{
	ConvertOdo();
	// send data via UART every 50 ms + a fraction of a microsec
	g_sentData = true;
	TCNT3 = 0x0000; // reset timer
}