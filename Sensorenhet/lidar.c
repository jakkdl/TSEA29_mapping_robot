#include "lidar.h"

void measure_lidar()
{
	PORTB &= ~(0x10); //pull PB4 low to start PWM reading from lidar F wont work since interrupts are queued when cli is used
	//interrupt on rising edge of PB5
	_delay_ms(1);
	cli();
	PORTB |= (1 << PORTB4);
	sei();
	PORTB &= ~(0x40); // pull PB6 low to start PWM reading from lidar B
	_delay_ms(1);
	cli();
	PORTB |= (1 << PORTB6);
	sei();
}

void ext_interupt_init()
{
	EICRA = (1<<ISC11) | (1 << ISC10) | (1 << ISC21); //enable interrupt on rising edge for B-pins and falling edge on C-pins
	PCICR = (1 << PCIE1) | (1 << PCIE2); // enable interrupts for B and C pins
	PCMSK2 = (1 << PCINT16) | (1 << PCINT17); // enable interrupts on only PC0 and PC1
	PCMSK1 = (1 << PCINT15) | (1 << PCINT13); // enable interrupts on only PB5 and PB7
}

void timer_init()
{
	TCNT1 = 0x0000;
	TCCR1B = (1 << CS11); // divide clock by 8 to get 2 tick every microsec
}

void timer_stop()
{
	TCCR1B = 0x00;
}