#include "adc.h"
void adc_init()
{
	ADMUX = (1 << REFS0); // use VCC as reference for conversion
	ADCSRA = (1 << ADEN) | (1 << ADIE) ; // Enable ADC and set it to do interrupt when completed
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); //prescale internal adc clock to land in 50-200 kHz range in this case 125kHz
	DIDR0 = (1 << ADC0D) | (1 << ADC1D) |(1 << ADC2D) |(1 << ADC3D) |(1 << ADC4D) | (1 << ADC5D) | (1 << ADC6D); // disable digital input for pins that are used as analog inputs
}

void start_adc()
{
	ADCSRA |= (1 << ADSC);
}

void next_input_pin()
{
	if (ADMUX == 0x43) // looped through all adc pins for IR sensors
	{
		ADMUX = 0x40; // return to ADC0 to be converted completed one loop
	}
	else
	{
		ADMUX++; // go to next input pin
		start_adc();
	}
}

uint8_t convert_voltage(double ADCVoltage)
// converts the input voltage to closest cm only between 8-80 cm
{
	double res = 0;
	res = 1/0.046053 * ADCVoltage;
	uint8_t result = round(res);
	return result;
}
