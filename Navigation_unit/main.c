#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdbool.h>
#include "pwm_timer.h"
#include "navigation_unit.h"
#include "nav_sensor_loop.h"
#include "nav_unit_com_interrupt_logic.h"
#include "../AVR_common/uart.h"
#include "../AVR_testing/test.h"

// sleep mode documentation at page 40

int main(void)
{
    Uart_Init();
#if __TEST__
    Test_run();
    return 0;
#endif

    PinInitPWM();
    //set_sleep_mode(<mode>)
    //switch busy wait to https://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
    struct data_packet sensor_data;
    struct data_packet com_data;
    sei();
    while(1)
    {
        _delay_us(1);
        if (Uart_Receive_0(&com_data))
        {
            communication_unit_interrupt(&com_data);
            com_data.bytes_read = 0xFF;
        }
        if (Uart_Receive_1(&sensor_data))
        {
            handle_sensor_data(&sensor_data);
            sensor_data.bytes_read = 0xFF;
        }
        if (g_SensorDataReady)
        {
            nav_main();
            g_SensorDataReady = false;
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

