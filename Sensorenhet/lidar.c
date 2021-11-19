#include "lidar.h"

void MeasureLidar()
{
	PORTB &= ~(0x10); //pull PB4 low to start PWM reading from lidar F wont work since interrupts are queued when cli is used
	//interrupt on rising edge of PB5
	_delay_ms(1);
	cli();
	PORTB |= (1 << PORTB4);
	sei();
	PORTB &= ~(0x10); // pull PB6 low to start PWM reading from lidar B
	_delay_ms(5);
	cli();
	PORTB |= (1 << PORTB4);
	sei();
}

void ExtInterruptInit()
{
	//EICRA = (1<<ISC11) | (1 << ISC10) | (1 << ISC21); //enable interrupt on rising edge for B-pins and falling edge on C-pins
	PCMSK2 = (1 << PCINT16) | (1 << PCINT17); // enable interrupts on only PC0 and PC1
	PCMSK1 = (1 << PCINT15) | (1 << PCINT13); // enable interrupts on only PB5 and PB7
	PCICR = (1 << PCIE1) | (1 << PCIE2); // enable interrupts for B and C pins
}

void TimerInit()
{
	TCNT1 = 0x0000;
	TCCR1B = (1 << CS11); // divide clock by 8 to get 2 tick every microsec
}

void MsTimerInit()
{
	TCNT3 = 0x0000;
	TCCR3B = (1 << CS32); // divide clock by 256 to get 3125 ticks per 50ms
	OCR3A = 0x0C35; // output compare on 3125
	TIMSK3 = (1 << OCIE3A); // enable interrupts for output compare with OCR3A register
}
void TimerStop()
{
	TCCR1B = 0x00;
}