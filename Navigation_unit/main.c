#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "pwm_timer.h"
#include "navigation_unit.h"
#include "nav_sensor_loop.h"
#include "nav_unit_com_interrupt_logic.h"
#include "../AVR_common/uart.h"
#include "../AVR_testing/test.h"

int main(void)
{
#if __TEST__
    Test_run();
#endif

    Uart_Init();
    PinInitPWM();
    sei();
    while(1)
    {
        _delay_us(1);
        if (g_SensorDataReady)
        {
            __asm("nop");
            g_SensorDataReady = false;
            nav_main();
        }
    }
}

ISR(TIMER0_OVF_vect)
{
    OCR0A = g_wheelSpeedLeft;
    OCR0B = g_wheelSpeedRight;
    PORTB = (g_wheelDirectionLeft << PORTB1)
        | (g_wheelDirectionRight << PORTB2);
}

ISR(BADISR_vect)
{
    // user code here
    __asm("nop");
}

