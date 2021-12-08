#include "gyro.h"
int16_t g_angle = 0;
int16_t MLXGyroVal()
{
	// Vin is 2.5 when still linear up down when positive/negative rotation max 4.5 min 0.5 at 300 degrees/s
	// AngularRate = (4/600) * VOut + 2.5;
	// double AngularRate = 0;
	// double MLXV = ( ADC * 5 )/ 1023;
	// AngularRate = round((MLXV - 2.5) * 150);
    
/*
    V = 0, ADC=0, -375 deg/s
    V = 0.5, ADC = 1024/10, -300 deg/s
    V = 2.5, ADC = 512, 0 deg/s
    V = 4.5, ADC = 1024*9/10, 300
    V = 5.0, ADC = 1024, 375
    */
    // convert ADC to degrees/second, then to our angle format/second, then
    // multiply with update time to get angle change in fractions of UINT16_MAX.
    // return (int16_t)round( ((double) ADC / 1024 * 750 - 375) / 360 * 65536 * TIME_BETWEEN_SEND / 1000);
	return ADC;
}
void MeasureMLX()
{
	ADMUX = 0x45; // enable ADC from MLX
	ADCSRA |= (1 << ADSC); // enable ADC
}