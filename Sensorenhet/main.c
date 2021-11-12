#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include "adc.h"
#include "lidar.h"

int interrupts = 0;
bool ReadingDone = true;
void start_reading()
{
	start_adc();
	measure_lidar();
}

void pin_init()
{
	// define output pins
	DDRB = (1 << PORTB4) | (1 << PORTB6);
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
    while (1) 
    {
		if(ReadingDone)
		{
			ReadingDone = false;
			start_reading();	
		}
    }
}

ISR(ADC_vect)
{
	int ADCVoltage = 0;
	uint8_t ADCLowBit = ADCL;
	uint16_t ADCRes = ADCH<<8 | ADCLowBit; // puts result of ADC in ADCRes
	ADCVoltage = ADCRes * 5 / 1024;
	// lookup distance from adc voltage
	//store value in correct place in memory
	//next_input_pin(); //update ADMUX
	// update memory for next ad conversion
	start_adc();
}

ISR(PCINT1_vect)
{
	timer_stop();
	uint16_t PWMTime = 0;
	long LidarDistance = 0;
	if (interrupts == 0)
	{
		timer_init();
		interrupts++;
	}
	else if (interrupts == 1)
	{
		PWMTime = TCNT1;
		LidarDistance = PWMTime / 2;
		// save distance in memory
		interrupts++;
	}
	else if (interrupts == 2)
	{
		timer_init();
		interrupts++;
	}
	else
	{
		PWMTime = TCNT1;
		LidarDistance = PWMTime / 2; // distance in cm
		// store distance in memory
		interrupts = 0;
	}
}