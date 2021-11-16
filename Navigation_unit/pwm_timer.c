#include "pwm_timer.h"

void PinInitPWM()
{
	DDRB = (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3) | (1 << PORTB4); // set PORTB1-4 as outputs
	PORTB = (1 << PORTB1) | (1 << PORTB2); // set DIR high as standard
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01); // set on bottom clear on compare match set fast pwm
	TIMSK0 = (1 << TOIE0); // for interrupt on overflow
	TCCR0B = (1 << CS01) | (1 << CS00); // prescale clk/64 for 980.4Hz PWM signal
}