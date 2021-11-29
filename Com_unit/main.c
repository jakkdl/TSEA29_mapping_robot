#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

#include "../AVR_common/robot.h"
#include "../AVR_common/uart.h"

bool packet;
struct data_packet currentPaket;

int main(void)
{	
	/*main loop in com unit does nothing as it just works as a pass thought no error handling*/
	UART_Init( 0 );
	UART_Init( 1 );
    sei();
    while(1){
		_delay_ms(100);
		
    };
}


ISR( USART0_RX_vect ) //should be the firefly connection
{
    cli(); //disable interrupts
    struct data_packet received = DATA_Receive( 0 );
	DATA_Transmit( 1, &received );
    sei(); //re enable interrupts

}


ISR( USART1_RX_vect ) //should be the firefly connection
{
    cli(); //disable interrupts
    struct data_packet received = DATA_Receive( 1 );
	DATA_Transmit( 0, &received );
    sei(); //re enable interrupts

}




