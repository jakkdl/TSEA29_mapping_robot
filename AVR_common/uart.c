#include <stdint.h>
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#define UART_BAUD 207 //8MHz system clock



/*
 *TODO test the code save the data somewhere
 */

void UART_Init(uint8_t interface)
{
    if (interface == 0)
    {
        /* Set baud rate see page 174 in avr documentation*/
        // set upper part
        UBRR0H = (uint8_t)(UART_BAUD>>8);
        // set lower part
        UBRR0L = (uint8_t)UART_BAUD;

        /* Enable receiver and transmitter */
        UCSR0B = (1<<RXEN0)|(1<<TXEN0);

        /* Set frame format: 8data, 1stop bit, 1 parity bit */
        UCSR0C = (3<<UCSZ00);
    }
    else
    {
        /* Set baud rate see page 174 in avr documentations*/
        // set upper part
        UBRR1H = (uint8_t)(UART_BAUD>>8);
        // set lower part
        UBRR1L = (uint8_t)UART_BAUD;

        /* Enable receiver and transmitter */
        UCSR1B = (1<<RXEN1)|(1<<TXEN1);

        /* Set frame format: 8data, 1stop bit, 1 parity bit */
        UCSR1C = (3<<UCSZ10);
    }
    /* 0_0_1_1_0_1_1_0*/
}

void UART_Transmit(uint8_t interface, uint8_t data )
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

//add interrupts to the transmit part of the UART transmit also this part has not been tested
void DATA_Transmit(uint8_t interface, struct data_packet paket)
{
    uint8_t header = (paket.address<<4) | (paket.byte_count<<1);
    UART_Transmit(interface, header);

    /*transmission of the data*/
    uint8_t i = 0;
    while ( i < paket.byte_count ){
        /* Wait for empty transmit buffer */
        while ( !( UCSR1A & (1<<UDRE1)) )
            ;
        UART_Transmit( interface, paket.bytes[i] );
        i = i + 1;
    }
}

uint8_t UART_Receive(uint8_t interface){
    /* this part need to be uppdate to become and isr or intrup driven reciver atleast so we dont sample all the time */
    if (interface == 0)
    {
        /* This part might not be need need to check*/
        /* Wait for data to be received */
        while ( !(UCSR0A & (1<<RXC0)) )
            ;
        /* Get and return received data from buffer */
        return UDR0;
    }
    else
    {
        /* This part might not be need need to check*/
        /* Wait for data to be received */
        while ( !(UCSR1A & (1<<RXC1)) )
            ;
        /* Get and return received data from buffer */
        return UDR1;
    }

}

struct data_packet DATA_Receive( uint8_t interface )
{
    //creat an instance of a new paket to return once called by the ISR
    data_packet ReceivedPaket;

    //receive the first byte that contain all the info we need for receive the rest
    uint8_t header = UART_Receive( interface );

    ReceivedPaket.address = (header>>4) & 0xF;
    ReceivedPaket.byte_count = (header>>2) & 0x7;

    /*receive the rest of the data*/
    uint8_t i = 0;
    while ( i < ReceivedPaket.byte_count){
        /* Wait for data to be received */
        /* do we wait 2x the time here or not? I asume the check always passes
         * in the other function making that check redundant */
        while ( !(UCSR1A & (1<<RXC1)) )
            ;
        uint8_t currentRecivedPaket = UART_Receive( interface );
        ReceivedPaket.bytes[i] = currentRecivedPaket;
    }
    return ReceivedPaket;
}

ISR( USART0_RX__vect )
{
    cli(); //disable interrupts
    data_packet Received;
    Received = DATA_Receive(0); //This data needs to be saved some where?
    sei(); //re enable interrupts
}

ISR( USART1_RX__vect )
{
    cli(); //disable interrupts
    data_packet Received;
    Received = DATA_Receive(1); //This data needs to be saved some where?
    sei(); //re enable interrupts
}
