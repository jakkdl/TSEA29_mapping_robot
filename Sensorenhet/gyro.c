#include <avr/interrupt.h>
#include "gyro.h"
#include "lidar.h"
#include "adc.h"
int32_t g_gyroFault;
uint16_t g_Vref;
bool g_startup = true;
int16_t MLXGyroVal(void)
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
	uint16_t temp = ADC;
	/*if (temp < 540 && temp > 525)
	{
		g_gyroFault += temp - 512;
		if(g_gyroFault % 2 != 0)
		{
			g_gyroFault++;
		}
		g_gyroFault /= 2;
	}*/
    return (int16_t)round( ((double) (temp-g_gyroFault) / 1024  * 750 - 375) / 360 * 65536 * TIME_BETWEEN_SEND / 1000);
	//return ADC - g_gyroFault;
}

#define SAMPLES 100
void GyroInit(void)
{
	sei();
	g_gyroFault = 0;
	_delay_ms(10);   
	for(int i = 0; i < SAMPLES; i++)
	{
		ADCRead(0x05);
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