#define F_CPU 16000000UL

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#include "../AVR_common/robot.h"
#include "../AVR_common/uart.h"

bool packet;
struct data_packet currentPaket;

int main(void)
{
    sei();
    UART_Init(1);
    while(1){
        if( packet == true ){
            packet = false;
            DATA_Transmit(1, &currentPaket);

        }
    };

}


ISR( USART0_RX_vect ) //should be the firefly connection
{
    cli(); //disable interrupts
    struct data_packet received = DATA_Receive(0);
    currentPaket = received;
    packet = true;
    sei(); //re enable interrupts

}




