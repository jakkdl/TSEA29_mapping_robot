#define F_CPU 16000000UL

#include "lidar.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void measure_lidar()
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