#include "gyro.h"

uint8_t g_angle = 0;

int16_t MLXGyroVal()
{
	// Vin is 2.5 when still linear up down when positive/negative rotation max 4.5 min 0.5 at 300 degrees/s
	// AngularRate = (4/600) * VOut + 2.5;

	int32_t AngularRate = 0; //
	double MLXV = ADC * 5 / 1024;
	AngularRate = round((MLXV - 2.5) * 150);
	return AngularRate * 0.05; // multiply by time in seconds it takes to get here now 50ms measure actual time
}
void StartMLX()
{
	ADMUX = 0x46; // enable ADC from MLX
	g_angle = 0;
	ADCSRA |= (1 << ADSC); // enable ADC
}