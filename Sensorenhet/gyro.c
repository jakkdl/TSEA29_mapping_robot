#include <avr/interrupt.h>
#include "gyro.h"
#include "lidar.h"
#include "adc.h"
int32_t g_gyroFault;
uint16_t g_Vref;
bool g_startup = true;
int16_t MLXGyroVal()
{
/*
    V = 0, ADC=0, -375 deg/s
    V = 0.5, ADC = 1024/10, -300 deg/s
    V = 2.5, ADC = 512, 0 deg/s
    V = 4.5, ADC = 1024*9/10, 300
    V = 5.0, ADC = 1024, 375
    */
    // convert ADC to degrees/second, then to our angle format/second, then
    // multiply with update time to get angle change in fractions of UINT16_MAX.
    return (int16_t)round( ((double) (ADC-g_gyroFault) / 1024  * 750 - 375) / 360 * 65536 * TIME_BETWEEN_SEND / 1000);
	//return ADC - g_gyroFault;
}

void MeasureMLX()
{
	ADMUX = 0x45; // enable ADC from MLX
	StartAdc();
}

#define SAMPLES 100
void GyroInit(void)
{
	sei();
	g_gyroFault = 0;
	_delay_ms(10);   
	for(int i = 0; i < SAMPLES; i++)
	{
		MeasureMLX();
		_delay_ms(1);
	}
	if (g_gyroFault % SAMPLES > SAMPLES/2)
	{
		g_gyroFault = g_gyroFault / SAMPLES + 1;
	}
	else
	{
		g_gyroFault /= SAMPLES;
	}
	g_startup = false;
	cli();
}