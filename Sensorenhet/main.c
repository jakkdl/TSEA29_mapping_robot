#define F_CPU 16000000UL

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "adc.h"
#include "lidar.h"

int LeftCount = 0;
int RightCount = 0;
bool ReadingDone = true;
bool ReadMLX = false;
/*
 * TODO:
 * implement storage in memory
 * implement angle rate integration
 * implement communication with other devices
 * move around functions to correct positions
 * testing functionality
 */
void start_reading()
{
	ReadingDone = false;
	start_adc();
	measure_lidar();
}

void pin_init()
{
	// define output pins
	DDRB =  (1 << PORTB4) | (1 << PORTB6);
	PORTB = (1 << PORTB4) | (1 << PORTB6);
	DDRA = (1 << PORTA4);
	PORTA = 0x00;
	DDRC = (1 << PORTC4);
	PORTC = 0x00;
	DDRD = (1 << PORTD0) | (1 << PORTD7); // D7 for debugging purposes
	PORTD = 0x00;
}
int main(void)
{
	pin_init();
	adc_init();
	timer_init();
	sei();
	start_reading();
    while (1) 
    {
		if(ReadingDone)
		{
			// send data;
			start_reading();	
		}
    }
}

int convert_odo(int val)
{
	// converts odo count to mm traveled;
	int res = 0;
	int openings = 40;
	res = round(val * 65 * M_PI / openings);
	return res;
}
ISR(ADC_vect)
{
	if (ReadMLX)
	{
		cli();
		int WantedAngle = 90; // placeholder should come from styrenhet
		MLX_gyro(WantedAngle, ReadMLX);
		sei();
	}
	else
	{
	double ADCVoltage = 0;
	uint8_t IRDistance = 0;
	uint8_t ADCLowBit = ADCL;
	uint16_t ADCRes = ADCH<<8 | ADCLowBit; // puts result of ADC in ADCRes
	ADCVoltage = ADCRes * 5 / 1024;
	IRDistance = convert_voltage(ADCVoltage);
	// store value in correct place in memory
	next_input_pin(); //update ADMUX
	// update memory for next ad conversion
	start_adc();
	}
}

ISR(PCINT1_vect)
{
	uint16_t PWMTime = 0;
	uint16_t LidarDistance = 0;
	if ((PORTB & 0x20) & !(PORTB & 0x10)) // if PB5 is high but not PB4
	{
		timer_init();
		while (PORTB & 0x20){}
		timer_stop();
		cli();
		PWMTime = TCNT1;
		sei();
		LidarDistance = PWMTime / 2;
	}
	else if ((PORTB & 0x80) & !(PORTB & 0x40)) // if PB7 is high but not PB6 
	{
		timer_init();
		while (PORTB & 0x80){}
		timer_stop();
		cli();
		PWMTime = TCNT1;
		sei();
		LidarDistance = PWMTime / 2;
	}
	// save lidar distance
}
ISR(PCINT2_vect)
{
	if(PORTC & 0x01)
	{
		LeftCount++;
	}
	else
	{
		RightCount++;
	}
}