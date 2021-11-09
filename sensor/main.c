/*
 * Test1.c
 *
 * Created: 2021-11-03 08:06:04
 * Author : klajo125
 */ 
#define F_CPU 20000000 // clock speed

#include <avr/io.h>
#include <avr/interrupt.h>

void ADC_init()
{
	ADMUX = (1 << REFS0); // use VCC as reference for conversion
	ADCSRA = (1 << ADEN) | (1 << ADIE); // Enable ADC and set it to do interrupt when completed
	// | (1 << ADPS0); | (1 << ADPS1); | (1 << ADPS2); for ADC prescaling
	DIDR0 = (1 << ADC0D) |(1 << ADC1D) |(1 << ADC2D) |(1 << ADC3D); // disable digital input for pins
}

void start_ADC()
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
		ADMUX &= ~(1 << MUX0);
	}
	else if (ADMUX == 0x02) // if read from ADC2 shift to ADC 3
	{
		ADMUX |= (1 << MUX0);
	}
	else if (ADMUX & 0x03) // if read from ADC 3 return to ADC0
	{
		ADMUX &= ~(1 << MUX2);
		ADMUX &= ~(1 << MUX1);
		ADMUX &= ~(1 << MUX0); // reset mux to ADC0
	}
	else // if not ADC1-3 shift to ADC1
	{
		ADMUX |= (1 << MUX0);
	}
	
}
int main(void)
{
	ADC_init();
	sei();
	start_ADC();
    while (1) 
    {
    }
}

ISR(ADC_vect)
{
	//store value in correct place in memory
	next_input_pin(); //update ADMUX 
	// update memory for next ad conversion
	ADC_init();
}
