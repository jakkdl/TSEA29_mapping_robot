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
void MeasureIR()
{
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
		res = ADCVoltage*-44.4 + 202.2;
	}
	else if ((ADCVoltage <= 2.3) && (ADCVoltage > 1.62))
	{
		res = ADCVoltage*-76.9+276.9;
	}
	else if ((ADCVoltage <= 1.62) && (ADCVoltage > 1.09))
	{
		res = ADCVoltage*-174.9+435.6;
	}
	else if ((ADCVoltage <= 1.09) && (ADCVoltage > 0.73))
	{
		res = ADCVoltage*-416.7+695.8;
	}
	else if ((ADCVoltage <= 0.73) && (ADCVoltage > 0.52))
	{
		res = ADCVoltage*-864.7+1027.4;
	}
	else if ((ADCVoltage <= 0.52) && (ADCVoltage >= 0.4))
	{
		res = ADCVoltage*-1973.7+1581.6;
	}
	else
	{
		return -1;
	}
	result = (uint16_t) round(res);
	return result;
}