#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "../AVR_common/uart.h"
#include "../AVR_testing/test.h"
#include "pwm_timer.h"
#include "navigation_unit.h"
#include "nav_sensor_loop.h"
#include "nav_unit_com_interrupt_logic.h"


int main(void)
{
#if __TEST__
    Test_run();
#endif
    // set up uart interrupts
    // DDRD |= (1 << PORTD1 | 1 << PORTD3);
    // PORTD = 0x00;
    // Don't know if those need to be set when working with UART

    UART_Init(COM_UNIT_INTERFACE, true, true);
    UART_Init(SENSOR_UNIT_INTERFACE, true, false);
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
	PORTB = (g_wheelDirectionLeft << PORTB1) | (g_wheelDirectionRight << PORTB2); // set DIR high as standard
}

ISR(BADISR_vect)
{
    // user code here
	__asm("nop");
}

ISR( USART0_RX_vect )
{
	//cli(); //disable interrupts
	struct data_packet received = DATA_Receive(0);
	//sei(); //re enable interrupts
	communication_unit_interrupt(&received);
}

ISR( USART1_RX_vect )
{
	//cli(); //disable interrupts
	struct data_packet received = DATA_Receive(1);
	
	//DATA_Transmit(0, &received);
	
	//sei(); //re enable interrupts
	handle_sensor_data(&received);
}
