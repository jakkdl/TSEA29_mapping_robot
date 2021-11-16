#include <stdint.h>
#include <xc.h>
#include <avr/io.h>
#include "uart.h"
#define UART_BAUD 207 //8MHz system clock



/*
 *none of the below code has been tested as of yet need to be tested but has been complied and
 *ineeds to be tested and more implemetetions are needed
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
void DATA_Transmit(uint8_t interface, dataPaket paket){
        
        /*transmits the header paket*/
		unsigned char header = paket.header;
        UART_Transmit(interface, header);

        /*transmists the rest of the data*/
        uint8_t i = 0;
        while ( i < 8 ){
            /* Wait for empty transmit buffer */
             while ( !( UCSR1A & (1<<UDRE1)) )
            ;
            UART_Transmit( interface, paket.datapaket[i] );
            i = i + 1;
        }
}

uint8_t UART_Receive(uint8_t interface){

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


//TODO add parity checkers and interrupts to this part also this has not been tested
dataPaket DATA_Receive( uint8_t interface ){
    dataPaket ReceivedPaket;
    ReceivedPaket.header =  UART_Receive( interface);
     /*transmits the rest of the data*/
    uint8_t i = 0;
    while ( i < 8){
        /* Wait for data to be received */
        while ( !(UCSR1A & (1<<RXC1)) )
            ;
        unsigned char currentRecivedPaket = UART_Receive( interface);
        ReceivedPaket.datapaket[i] = currentRecivedPaket;
    }
	return ReceivedPaket;
}


//I have no idea if this part will or works at all but something like this could be used check back when trying to compile in atmel
void parityError( uint8_t interface, dataPaket paket ){
    byteUnion headerbyte;
	headerbyte.b = paket.header;
    dataPaket erroPaket;
    erroPaket.header = 0xF4; //correct bits need to be set here 
    erroPaket.datapaket[0] = ( 3<<headerbyte.b7 ) | (2<<headerbyte.b6) | (1<<headerbyte.b5) | ( headerbyte.b4 );
    DATA_Transmit(  interface , erroPaket );
}

