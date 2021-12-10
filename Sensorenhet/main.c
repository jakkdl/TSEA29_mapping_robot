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
#define OPENINGS 40
uint8_t g_leftCount = 0;
uint8_t g_rightCount = 0;
uint16_t g_lidarDistance = 0; // distance in mm
bool g_readingDone = true;
bool g_sendData = false;

#define WHEEL_DIAMETER 62

struct sensor_data data;

//TODO: testing functionality: MLX gyro

void StartReading()
{
    g_readingDone = false;
    g_sendData = false;
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
	AdcInit();
	GyroInit();
	ExtInterruptInit();
    Uart_Init();
	TimerInit();
    MsTimerInit();
	sei();
    
    StartReading();
    while (1)
    {
        _delay_us(1);
        if(g_readingDone && g_sendData)
        {
            TCNT3 = 0x0000; // reset timer
            SendData();
            //_delay_ms(1000);
            StartReading();
        }
    }
}
struct data_packet packet;
void SendData()
{
    packet.byte_count = 2;

    uint16_t* value = (uint16_t*) &data;
    for (int i=0; i < 7; ++i)
    {
        packet.address = i;
        packet.bytes[0] = Uint16ToByte0(*value);
        packet.bytes[1] = Uint16ToByte1(*value);
        Uart_Send_0(&packet);
        ++value;
    }
    packet.address = ODOMETER;
    packet.bytes[0] = data.odometer_left;
    packet.bytes[1] = data.odometer_right;
    Uart_Send_0(&packet);
}



void ConvertOdo()
{
    // converts odo count to mm traveled;
    data.odometer_left = round(g_leftCount * WHEEL_DIAMETER * M_PI / OPENINGS); // max is 50 mm /cycle / 10 pegs
    data.odometer_right = round(g_rightCount * WHEEL_DIAMETER * M_PI / OPENINGS); // const 5.105088
    g_rightCount = 0;
    g_leftCount = 0;
}

ISR(ADC_vect)
{
    if (ADMUX == 0x45)
    {
		if(g_startup)
		{
			int16_t temp = ADC - 512;
			g_gyroFault += temp;
		}
		else
		{
			data.gyro = -1*MLXGyroVal();
			g_readingDone = true;
		}
    }
    else // reading from IR
    {
        double ADCVoltage = 0;
        uint16_t IRDistance = 0;
        uint8_t ADCLowBit = ADCL;
        double ADCRes = ADCH<<8 | ADCLowBit; // puts result of ADC in ADCRes
        ADCRes = ADCRes * 5;
        ADCVoltage = ADCRes / 1024;
        IRDistance = ConvertVoltage(ADCVoltage);
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

    // send data via UART every 50 ms + a fraction of a microsec
    g_sendData = true;
    ConvertOdo();
}

ISR(BADISR_vect)
{
    // user code here
}
