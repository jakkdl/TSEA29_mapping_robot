// Main function when running normally, i.e. not running unit tests
// Will probably be pretty minimal
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pwm_timer.h"

uint8_t g_leftSpeed = 0;
uint8_t g_rightSpeed = 0;

int main(void)
{
    // set up uart interrupts
    DDRD |= (1 << PORTD1 | 1 << PORTD3);
    PORTD = 0x00;
    UART_Init(0);
    UART_Init(1);

	//PinInitPWM();
    sei();
    while(1)
	{
	}

#if __TEST__
    Test_run();
#endif
}

ISR(TIMER0_OVF_vect)
{
	OCR0A = g_leftSpeed;
	OCR0B = g_rightSpeed;
}
