#define F_CPU 16000000UL

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <util/delay.h>
#include "adc.h"
#include "lidar.h"
#include "gyro.h"
#include "../AVR_common/sensors.h"
#include "../AVR_common/robot.h"
#include "../AVR_common/uart.h"

void SendData();
uint8_t OPENINGS = 40;
uint8_t g_leftCount = 0;
uint8_t g_rightCount = 0;
uint16_t g_lidarDistance = 0; // distance in mm
bool g_readingDone = true;
bool g_sendData = false;

struct sensor_data data;
/*
 * TODO:
 * implement communication with other devices
 * move around functions to correct positions
 * testing functionality:
 * MLX gyro
 */

void StartReading()
{
	g_readingDone = false;
	MeasureIR();
	data.lidar_forward = MeasureLidarFront();
	data.lidar_backward = MeasureLidarBack();
	MeasureMLX();
}

void PinInit()
{
	// define output pins
	DDRB |= (1 << PORTB4) | (1 << PORTB6);
	DDRB &= ~(1 << PORTB5);
	DDRB &= ~(1 << PORTB7);
	PORTB = 0x00;
	PORTB |= (1 << PORTB4) | (1 << PORTB6);
	DDRA |= (1 << PORTA4);
	PORTA = 0x00;
	DDRC |= (1 << PORTC4);
	PORTC = 0x00;
	DDRD |= (1 << PORTD1);
	PORTD = 0x00;
}

int main(void)
{
	PinInit();
	TimerInit();
	ExtInterruptInit();
	UART_Init(0);
	MsTimerInit();
	sei();
	StartReading();
    while (1)
    {
		if(g_readingDone && g_sendData)
		{
			SendData();
			StartReading();
			//_delay_ms(100);
		}
    }
}

void SendData()
{
	struct data_packet packet;
	packet.address = IR_LEFTFRONT;
	packet.byte_count = 2;
	packet.bytes[0] = Uint16ToByte0(data.ir_leftfront);
	packet.bytes[1] = Uint16ToByte1(data.ir_leftfront);
	DATA_Transmit(0, &packet);
}

void ConvertOdo()
{
	// converts odo count to mm traveled;
	data.odometer_left = round(g_leftCount * 65 * M_PI / OPENINGS); // max is 50 mm /cycle / 10 pegs
	data.odometer_right = round(g_rightCount * 65 * M_PI / OPENINGS); // const 5.105088
	g_rightCount = 0;
	g_leftCount = 0;
}

ISR(ADC_vect)
{
	if ((ADMUX & (1 << PORTB3)) && (ADMUX & 1 << PORTB2)) // reading from MLX
	{
		cli();
		g_angle += MLXGyroVal();
		sei();
		data.gyro = g_angle;
		g_readingDone = true;
	}
	else // reading from IR
	{
		double ADCVoltage = 0;
		uint16_t IRDistance = 0;
		uint8_t ADCLowBit = ADCL;
		double ADCRes = ADCH<<8 | ADCLowBit; // puts result of ADC in ADCRes
		ADCRes = ADCRes * 5;
		ADCVoltage = ADCRes / 1024;
		cli();
		IRDistance = ConvertVoltage(ADCVoltage);
		sei();
		switch (ADMUX)
		{
			case 0x40:
				data.ir_leftfront = IRDistance;
				break;
			case 0x41:
				data.ir_leftback = IRDistance;
				break;
			case 0x42:
				data.ir_rightfront = IRDistance;
				break;
			case 0x43:
				data.ir_rightback = IRDistance;
		}
	}
}

ISR(INT0_vect)
{
	g_leftCount++;
}

ISR(INT1_vect)
{
	g_rightCount++;
}

ISR(TIMER3_COMPA_vect)
{
	ConvertOdo();
	// send data via UART every 50 ms + a fraction of a microsec
	g_sendData = true;
	TCNT3 = 0x0000; // reset timer
}