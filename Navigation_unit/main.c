#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "../AVR_common/uart.h"
#include "pwm_timer.h"
#include "navigation_unit.h"


int main(void)
{
    // set up uart interrupts
    // DDRD |= (1 << PORTD1 | 1 << PORTD3);
    // PORTD = 0x00;
    // Don't know if those need to be set when working with UART

    UART_Init(0);
    UART_Init(1);



    // timer init?
    // TCNT1 = 0x0000;
    // TCCR1B = (1 << CS11); // divide clock by 8 to get 2 tick every microsec

    PinInitPWM();
    sei();

#if __TEST__
    Test_run();
#endif
    //g_wheelSpeedLeft = 0x30;
    //_delay_ms(1);
    g_wheelSpeedLeft = 0x00;
    while(1)
    {
    }
}

ISR(TIMER0_OVF_vect)
{
    OCR0A = g_wheelSpeedLeft;
    OCR0B = g_wheelSpeedRight;
}
