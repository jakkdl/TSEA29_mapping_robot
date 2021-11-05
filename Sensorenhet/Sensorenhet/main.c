/*
 * GccApplication1.c
 *
 * Created: 2021-11-05 08:37:58
 * Author : simda769
 */ 
#define F_CPU 20000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void next_input_pin();
void start_adc();
void adc_init();
void pin_init();

int main(void)
{
    /* Replace with your application code */
	pin_init();
	adc_init();
	sei();
	start_adc();
	//PORTD = (1 << PIND7);
	//DDRD = (1 << PIND7);
    while (1) 
    {
		//_delay_ms(100);
    }
}
void pin_init()
{
	// define output pins
	DDRB = (1 << DDB4) | (1 << DDB6);
	DDRA = (1 << DDA4);
	DDRC = (1 << DDC4);
	DDRD = (1 << DDD0) | (1 << DDD7); // D7 for debugging purposes
}

void adc_init()
{
	ADMUX = (1 << REFS0); // use VCC as reference for conversion
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // Enable ADC and set it to do interrupt when completed
	//  last 3 for ADC prescaling
	DIDR0 = (1 << ADC0D) | (1 << ADC1D) |(1 << ADC2D) |(1 << ADC3D) |(1 << ADC4D) | (1 << ADC5D) | (1 << ADC6D); // disable digital input for pins
}

void start_adc()
{
	ADCSRA |= (1 << ADSC);
}

void next_input_pin()
{
	//ADMUX++; // probably not ok
	//might be more correct
	if (ADMUX & 0x01) //if read from ADC1 shift to ADC 2
	{
		ADMUX |= (1 << MUX1);
		ADMUX &= (0 << MUX0);
	}
	else if (ADMUX == 0x02) // if read from ADC2 shift to ADC 3
	{
		ADMUX |= (1 << MUX0);
	}
	else if (ADMUX & 0x03) // if read from ADC 3 return to ADC0
	{
		ADMUX &= (0 << MUX2);
		ADMUX &= (0 << MUX1);
		ADMUX &= (0 << MUX0); // reset mux to ADC0
	}
	else // if not ADC1-3 shift to ADC1
	{
		ADMUX |= (1 << MUX0);
	}
	
}

ISR(ADC_vect)
{
	//store value in correct place in memory
	//next_input_pin(); //update ADMUX
	PORTD = (1 << PIND7);
	// update memory for next ad conversion
	start_adc();
	_delay_ms(100);
	PORTD = (0 << PIND7);
	_delay_ms(100);
}
