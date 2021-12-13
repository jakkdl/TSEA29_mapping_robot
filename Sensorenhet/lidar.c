#include "lidar.h"

uint16_t MeasureLidarFront(void)
{
	// PWMsignal is max 4ms
	// result of this function varies greatly when < 30cm stabilizes after 3 measurments
	uint16_t PWMTime = 0;
	uint16_t firstTime = 0;
	uint16_t lidarF = 0;
	cli();
	PORTB &= ~(0x10); //pull PB4 low to start PWM reading from lidar F
	while(!(PINB & (1 << PINB4)))
	{
		if ((PINB & (1 << PINB7)) && !(PINB & (1 << PINB4))) // if PB7 is high but not PB4 read pwm time
		{
			// read clock
			firstTime = TCNT1;
			while (PINB & (1 << PINB7))
			;
			//read clock
			PWMTime = TCNT1;
			PORTB |= (1 << PORTB4);
			if(PWMTime < firstTime)
			{
				PWMTime += (0xFFFF - firstTime);
			}
			else
			{
				PWMTime -= firstTime;
			}
			lidarF = PWMTime / 2;
		}
	}
	if(lidarF > 1000)
	{
		lidarF += 35;
	}
	sei();
	return lidarF;
}

uint16_t MeasureLidarBack(void)
{
	uint16_t PWMTime = 0;
	uint16_t firstTime = 0;
	uint16_t lidarB = 0;
	cli();
	PORTB &= ~(0x40); // pull PB6 low to start PWM reading from lidar B
	
	while(!(PINB & (1 << PINB6)))
	{
		if ((PINB & (1 << PINB5)) && !(PINB & (1 << PINB6))) // if PB5 is high but not PB6
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
		}
	}
	if (lidarB > 1000)
	{
		lidarB += 35;
	}
	sei();
	return lidarB;
}

void ExtInterruptInit(void)
{
	EICRA = (1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10);
	EIMSK = (1 << INT0) | (1 << INT1);
}

void TimerInit(void)
{
	TCNT1 = 0x0000;
	TCCR1B = (1 << CS11); // divide clock by 8 to get 2 tick every microsec
}


#define F 62.5
void MsTimerInit(void)
{
	TCNT3 = 0x0000;
	TCCR3B = (1 << CS32); // divide clock by 256 to get 3125 ticks per 50ms
	OCR3A = F * TIME_BETWEEN_SEND; // output compare on 3125 0C35
	TIMSK3 = (1 << OCIE3A); // enable interrupts for output compare with OCR3A register
}