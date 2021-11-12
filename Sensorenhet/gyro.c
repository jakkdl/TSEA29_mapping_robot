#include "gyro.h"

void MLX_gyro(int WantedAngle, bool ReadMLX)
{
	// Vin is 2.5 when still linear up down when positive/negative rotation max 4.5 min 0.5 at 300 degrees/s
	// AngularRate = (4/600) * VOut + 2.5;
	ADMUX = 0x46; // enable ADC from ADC6
	int fault = 1; // tolerable fault in degrees
	int AngularRate = 0;
	int Angle = 0;
	while (ReadMLX)
	{
		start_adc();
		double MLXV = ADC * 5 / 1024;
		AngularRate = round((MLXV - 2.5) * 150);
		// do integration to calculate angle
		Angle += AngularRate * 0.000104; // TimeForLoop in S
		if ((Angle >= WantedAngle + fault) | (Angle <= WantedAngle - fault))
		{
			ReadMLX = false;
		}
	}
	// store angle maybe
}
