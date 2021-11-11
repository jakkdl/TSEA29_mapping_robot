/*
 * GccApplication1.c
 *
 * Created: 2021-11-05 08:37:58
 * Author : simda769
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void next_input_pin();
void start_adc();
void adc_init();
void pin_init();
void start_reading();
bool ReadingDone;
int interrupts = 0;
int main(void)
{
	pin_init();
	adc_init();
	timer_init();
	sei();
	start_reading();
    while (1) 
    {
		if (ReadingDone)
		{
			start_reading():
		}
    }
}
void start_reading()
{
	start_adc();
	measure_lidar();
}
void measure_lidar();
{
	cli();
	PORTB &= ~(0x10); //pull PB4 low to start pwm reading from lidar F
	sei();
	//interrupt on rising edge of PB5 then do
	_delay_ms(1);
	cli();
	PORTB |= (1 << PORTB4);
	sei();
	cli();
	PORTB &= ~(0x40); // pull PB6 low to start PWM reading from lidat B
	sei();
	_delay_ms(1);
	cli();
	PORTB |= (1 << PORTB6);
	sei();
}
void ext_interupt_init()
{
	EICRA = (1<<ISC10); //enable interupt on rising edge
	PCMSK1 = (1 << PCINT15) | (1 << PCINT14); // enable interrupts on PB5 and PB7
}
void timer_init()
{
	TCNT1 = 0x0000;
	TCCR1B = (1 << CS20) |(1 << CS21); // divide clock by 64 to get 1 tick every 2 microsec
}
void timer_stop()
{
	TCCR1B = 0x00;
}
void pin_init()
{
	// define output pins
	DDRB = (1 << PORTB4) | (1 << PORTB6);
	PORTB = (1 << PORTB4) | (1 << PORTB6);
	DDRA = (1 << PORTA4);
	PORTA = 0x00;
	DDRC = (1 << PORTC4);
	PORTC = 0x00;
	DDRD = (1 << PORTD0) | (1 << PORTD7); // D7 for debugging purposes
	PORTD = 0x00;
}

void adc_init()
{
	ADMUX = (1 << REFS0); // use VCC as reference for conversion
	ADCSRA = (1 << ADEN) | (1 << ADIE) ; // Enable ADC and set it to do interrupt when completed
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); //prescale internal adc clock to land in 50-200 kHz range in this case 125kHz
	DIDR0 = (1 << ADC0D) | (1 << ADC1D) |(1 << ADC2D) |(1 << ADC3D) |(1 << ADC4D) | (1 << ADC5D) | (1 << ADC6D); // disable digital input for pins that are used as analog inputs
}

void start_adc()
{
	ADCSRA |= (1 << ADSC);
}

void next_input_pin()
{
	if (ADMUX == 0x43) // looped through all adc pins for IR sensors
	{
		ADMUX = 0x40; // return to ADC0 to be converted completed one loop
	}
	else
	{
		ADMUX++; // go to next input pin
		start_adc();
	}
}

ISR(ADC_vect)
{
	uint8_t ADCLowBit = ADCL;
	uint16_t ADCRes = ADCH<<8 | ADCLowBit; // puts result of ADC in ADCRes
	//store value in correct place in memory
	next_input_pin(); //update ADMUX
	// update memory for next ad conversion
}
ISR(PCINT1_vect)
{
	timer_stop();
	uint16_t PWMTime = 0;
	long LidarDistance = 0;
	if (interrupts == 0)
	{
		timer_init();
		interrupts++
	}
	else if (interrupts == 1)
	{
		PWMTime = TCNT1;
		LidarDistance = PWMTime / 2;
		// save distance in memory
		interrupts++;
	}
	else if (interrupts == 2)
	{
		timer_init();
		interrupts++;
	}
	else
	{
		PWMTime = TCNT1;
		LidarDistance = PWMTime / 2; // distance in cm
		// store distance in memory
		interrupts = 0;
	}
}