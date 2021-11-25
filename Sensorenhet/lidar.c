#include "lidar.h"

void MeasureLidar()
{
	PORTB &= ~(0x10); //pull PB4 low to start PWM reading from lidar F
	_delay_ms(1);
	uint16_t PWMTime = 0;
	uint16_t firstTime = 0;
	uint16_t lidarF = 0;
	uint16_t lidarB = 0;
	while(PINB & ~(1 << PINB4))
	{
		if ((PINB & (1 << PINB7)) && !(PINB & (1 << PINB4))) // if PB7 is high but not PB4 read pwm time
		{
			firstTime = TCNT1;
			while (PINB & (1 << PINB7))
			;
			//read clock
			PWMTime = TCNT1;
			PORTB |= (1 << PINB4);
			uint16_t temp = PWMTime;
			if(PWMTime < firstTime)
			{
				PWMTime += (0xFFFF - firstTime);
			}
			else
			{
				PWMTime -= firstTime;
			}
			lidarF = PWMTime / 2;
			lidarF -= 30;	// front
		}
	}
	// works decently but sometimes wrong val (very)
	// PWMsignal is 4ms
	/*PORTB &= ~(0x40); // pull PB6 low to start PWM reading from lidar B
	_delay_ms(1);*/
	PWMTime = 0;
	firstTime = 0;
	while(PINB & ~(1 << PINB6))
	{
		if ((PINB & (1 << PINB5)) && !(PINB & (1 << PINB6))) // if PB5 is high but not PB4
		{
			//read clock
			firstTime = TCNT1;
			while (PINB & (1 << PINB5))
			;
			//read clock
			PWMTime = TCNT1;
			PORTB |= (1 << PORTB6);
			if(PWMTime < firstTime)
			{
				PWMTime += (0xFFFF - firstTime);
			}
			else
			{
				PWMTime -= firstTime;
			}
			lidarB= PWMTime / 2;
			lidarB -= 30; // back
		}
	}
}

void ExtInterruptInit()
{
	//EICRA = (1<<ISC11) | (1 << ISC10) | (1 << ISC21); //enable interrupt on rising edge for B-pins and falling edge on C-pins
	//PCMSK2 = (1 << PCINT16) | (1 << PCINT17); // enable interrupts on only PC0 and PC1
	//PCMSK1 = (1 << PCINT15) | (1 << PCINT13); // enable interrupts on only PB5 and PB7
	//PCICR = (1 << PCIE1) | (1 << PCIE2); // enable interrupts for B and C pins
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