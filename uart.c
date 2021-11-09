#include <avr/io.h>
#include "uart.h"

void UART_Init(uint8_t interface)
{
    if (interface == 0)
    {
        /* Set baud rate */
        UBRR0H = (unsigned char)(UART_BAUD>>8);
        UBRR0L = (unsigned char)UART_BAUD;

        /* Enable receiver and transmitter */
        UCSR0B = (1<<RXEN0)|(1<<TXEN0);

        /* Set frame format: 8data, 2stop bit */
        //UCSRnC = (1<<USBSn)|(3<<UCSZn0);

        /* Set frame format: 8data, 1stop bit, 1 parity bit */
        UCSR0C = (3<<UCSZ00);
    }
    else
    {
        /* Set baud rate */
        UBRR1H = (unsigned char)(UART_BAUD>>8);
        UBRR1L = (unsigned char)UART_BAUD;

        /* Enable receiver and transmitter */
        UCSR1B = (1<<RXEN1)|(1<<TXEN1);

        /* Set frame format: 8data, 2stop bit */
        //UCSRnC = (1<<USBSn)|(3<<UCSZn0);

        /* Set frame format: 8data, 1stop bit, 1 parity bit */
        UCSR1C = (3<<UCSZ10);
    }
}

void UART_Transmit(uint8_t interface, unsigned char data )
{
    if (interface == 0)
    {
        /* Wait for empty transmit buffer */
        while ( !( UCSR0A & (1<<UDRE0)) )
            ;
        /* Put data into buffer, sends the data */
        UDR0 = data;
    }
    else
    {
        /* Wait for empty transmit buffer */
        while ( !( UCSR1A & (1<<UDRE1)) )
            ;
        /* Put data into buffer, sends the data */
        UDR1 = data;
    }

}

unsigned char UART_Receive(uint8_t interface)
{
    if (interface == 0)
    {
        /* Wait for data to be received */
        while ( !(UCSR0A & (1<<RXC0)) )
            ;
        /* Get and return received data from buffer */
        return UDR0;
    }
    else
    {
        /* Wait for data to be received */
        while ( !(UCSR1A & (1<<RXC1)) )
            ;
        /* Get and return received data from buffer */
        return UDR1;
    }

}
