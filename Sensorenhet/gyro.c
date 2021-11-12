#include "gyro.h"

int MLX_gyro()
{
	// Vin is 2.5 when still linear up down when positive/negative rotation max 4.5 min 0.5 at 300 degrees/s
	// AngularRate = (4/600) * VOut + 2.5;
	ADMUX = 0x46; // enable ADC from ADC6
	int AngularRate = 0;
	double MLXV = ADC * 5 / 1024;
	AngularRate = round((MLXV - 2.5) * 150);
	// do integration to calculate angle
	// store angle maybe
	_delay_ms(10);
	return AngularRate *  0.010104;
}
