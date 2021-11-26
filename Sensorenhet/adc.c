#include "adc.h"

bool g_IRDone = false;

void AdcInit()
{
	ADMUX = (1 << REFS0); // use VCC as reference for conversion
	ADCSRA = (1 << ADEN) | (1 << ADIE) ; // Enable ADC and set it to do interrupt when completed
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); //prescale internal adc clock to land in 50-200 kHz range in this case 125kHz
	DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << ADC5D) | (1 << ADC6D); // disable digital input for pins that are used as analog inputs
}

void StartAdc()
{
	g_IRDone = false;
	ADCSRA |= (1 << ADSC);
}

void NextInputPin()
{
	if ((ADMUX & (1 << PORTB1)) && (ADMUX & (1 << PORTB0))) // looped through all adc pins for IR sensors
	{
		ADMUX = 0x40; // return to ADC0 to be converted completed one loop
		g_IRDone = true;
	}
	else
	{
		ADMUX++; // go to next input pin
		StartAdc();
	}
}

uint16_t ConvertVoltage(double ADCVoltage)
// converts the input voltage to closest cm only between 8-80 cm
{
	double res = 0;
	uint16_t result = 0;
	if(ADCVoltage > 2.8)
	{
		return 0;
	}
	else if (ADCVoltage < 0.4)
	{
		return -1;
	}
	else if ((ADCVoltage <= 2.8) && (ADCVoltage > 2.3))
	{
		res = ADCVoltage*-4.44 + 20.22;
	}
	else if ((ADCVoltage <= 2.3) && (ADCVoltage > 1.62))
	{
		res = ADCVoltage*-7.69+27.69;
	}
	else if ((ADCVoltage <= 1.62) && (ADCVoltage > 1.09))
	{
		res = ADCVoltage*-17.49+43.56;
	}
	else if ((ADCVoltage <= 1.09) && (ADCVoltage > 0.73))
	{
		res = ADCVoltage*-41.67*69.58;
	}
	else if ((ADCVoltage <= 0.73) && (ADCVoltage > 0.52))
	{
		res = ADCVoltage*-86.47+102.74;
	}
	else if ((ADCVoltage <= 0.52) && (ADCVoltage >= 0.4))
	{
		res = ADCVoltage*-197.37+158.16;
	}
	else
	{
		return -1;
	}
	result = (uint16_t) round(res);
	return result;
}