#define F_CPU 16000000UL

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "adc.h"
#include "lidar.h"
#include "gyro.h"
#include <util/delay.h>

uint8_t OPENINGS = 40;
uint8_t g_leftCount = 0;
uint8_t g_rightCount = 0;
uint16_t g_lidarDistance = 0; // distance in mm
bool g_readingDone = true;
bool g_sentData = true;
/*
 * TODO:
 * implement storage in memory where all data is stored for sending
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
	_delay_ms(1);
	NextInputPin();
	_delay_ms(1);
	NextInputPin();
	_delay_ms(1);
	NextInputPin();
	_delay_ms(1);
	NextInputPin();
	//MeasureLidar();
	//while(!g_IRDone){}
	//StartMLX();
	//g_readingDone = true;
}

void PinInit()
{
	// define output pins
	DDRB |= (1 << PORTB4) | (1 << PORTB6);
	DDRB &= ~(1 << PORTB5);
	DDRB &= ~(1 << PORTB7);
	PORTB = 0x00;
	PORTB |= (1 << PORTB4) | (1 << PORTB6);
	DDRA |= (1 << PORTA4);
	PORTA = 0x00;
	DDRC |= (1 << PORTC4);
	PORTC = 0x00;
	DDRD |= (1 << PORTD0);
	PORTD = 0x00;
}
int main(void)
{
	PinInit();
	TimerInit();
	ExtInterruptInit();
	//MsTimerInit();
	sei();
	StartReading();
    while (1) 
    {
		if(true)
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
	if ((ADMUX & (1 << PORTB3)) && (ADMUX & 1 << PORTB2)) // reading from MLX
	{
		cli();
		g_angle += MLXGyroVal();
		sei();
	}
	else // reading from IR
	{
		double ADCVoltage = 0;
		uint16_t IRDistance = 0;
		uint8_t ADCLowBit = ADCL;
		double ADCRes = ADCH<<8 | ADCLowBit; // puts result of ADC in ADCRes
		ADCRes = ADCRes * 5;
		ADCVoltage = ADCRes / 1024;
		cli();
		IRDistance = ConvertVoltage(ADCVoltage);
		sei();
		// store value in correct place in memory
		//NextInputPin(); //update ADMUX
		// update memory for next ad conversion
	}
}

ISR(PCINT1_vect)
{
	cli();
	uint16_t PWMTime = 0;
	uint16_t firstTime = 0;
	// PWMsignal is 4ms
	if ((PINB & (1 << PINB5)) && !(PINB & (1 << PINB6))) // if PB5 is high but not PB4
	{
		//read clock
		firstTime = TCNT1;
		while (PINB & (1 << PINB5)){}
		//read clock
		PWMTime = TCNT1;
		if(TIFR1 & 0x01)
		{
			PWMTime += (0xFFFF - firstTime);
		}
		else
		{
			PWMTime -= firstTime;
		}
		g_lidarDistance = PWMTime / 2;
	}
	else if ((PINB & (1 << PINB7)) && !(PINB & (1 << PINB4))) // if PB7 is high but not PB4 read pwm time
	{
		firstTime = TCNT1;
		while (PINB & (1 << PINB7)){}
		//read clock
		PWMTime = TCNT1;
		if(TIFR1 & 0x01)
		{
			PWMTime += (0xFFFF - firstTime);
		}
		else
		{
			PWMTime -= firstTime; // PWMTime is about 20-50 too big
		}
		g_lidarDistance = PWMTime / 2;
	}
	sei();
	// save lidar distance
}
ISR(PCINT2_vect)
{
	if(PINC & (1 << PINC0))
	{
		g_leftCount++;
	}
	else if (PINC & (1 << PINC1))
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