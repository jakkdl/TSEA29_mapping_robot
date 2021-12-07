#include "gyro.h"
int16_t g_angle = 0;
int16_t MLXGyroVal()
{
	// Vin is 2.5 when still linear up down when positive/negative rotation max 4.5 min 0.5 at 300 degrees/s
	// AngularRate = (4/600) * VOut + 2.5;
	// double AngularRate = 0;
	// double MLXV = ( ADC * 5 )/ 1023;
	// AngularRate = round((MLXV - 2.5) * 150);
	
	
	
	double ADCVoltage = 0;
    int16_t AR = 0;
    double ADCRes = ADCH<<8 | ADCL; // puts result of ADC in ADCRes
    ADCRes = ADCRes * 5;
    ADCVoltage = ADCRes / 1024;
        
	return ADCVoltage; // multiply by time in seconds it takes to get here now 50ms measure actual time 
}
void MeasureMLX()
{
	ADMUX = 0x45; // enable ADC from MLX
	g_angle = 0;
	ADCSRA |= (1 << ADSC); // enable ADC
}