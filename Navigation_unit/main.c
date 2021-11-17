// Main function when running normally, i.e. not running unit tests
// Will probably be pretty minimal
#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pwm_timer.h"
uint8_t g_leftSpeed = 0;
uint8_t g_rightSpeed = 0;
int main(void)
{
    // set up uart interrupts
	//PinInitPWM();
	sei();
    while(1)
	{
	}
}
ISR(TIMER0_OVF_vect)
{
	OCR0A = g_leftSpeed;
	OCR0B = g_rightSpeed;
}
