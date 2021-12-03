#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#define UART_BAUD 8 //16MHz system clock 115.2k baud
//#define UART_BAUD 103 // 9.6k baud

/*
 *This code now work to make parity work we need to set setting on firefly so
 *8bit + 1 stop + even parity at 115,200 baud is the current setting that works i have adjusted to code
 */



//interrupts for tx should not be used as the restart the atmega for now
void UART_Init(uint8_t interface, bool rx, bool tx)
{
	/*this function set the correct bits in the various regs current baud is 9600 at clock at 16Mhz */
    if (interface == 0)
    {
        /* Set baud rate see page 174 in avr documentation*/
        // set upper part
        UBRR0H = (uint8_t)(UART_BAUD>>8);
        // set lower part
        UBRR0L = (uint8_t)UART_BAUD;

        /* Enable receiver and transmitter and tx and rx interrupts */
		UCSR0B = 0;
		if (tx)
		{
			UCSR0B |= (0<<TXCIE0) | (1<<TXEN0);
		}
		if (rx)
		{
			UCSR0B |= (1<<RXCIE0) | (1<<RXEN0);
		}
        /* Set frame format: 8data, 1stop bit, no parity bit */
        UCSR0C =  (0<<USBS0) |(1<<UPM01) | (3<<UCSZ00);
    }
    else
    {
        /* Set baud rate see page 174 in avr documentations*/
        // set upper part
        UBRR1H = (uint8_t)(UART_BAUD>>8);
        // set lower part
        UBRR1L = (uint8_t)UART_BAUD;

        /* Enable receiver and transmitter */
		UCSR1B = 0;
		if (tx)
		{
			UCSR1B |= (0<<TXCIE1) | (1<<TXEN1);
		}
		if (rx)
		{
			UCSR1B |= (1<<RXCIE1) | (1<<RXEN1);
		}
        /* Set frame format: 8data, 1stop bit, no parity bit */
		UCSR1C =  (0<<USBS1) | (1<<UPM11) | (3<<UCSZ10);
    }
}


//interrupts for tx should not be used as the restart the atmega for now
void UART_Transmit(uint8_t interface, uint8_t data )
{
	/*this function is the basic UART send function that puts data in the UDRn, atmega does the rest*/
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
void DATA_Transmit(uint8_t interface, struct data_packet *paket)
{
	/*This function sends a struct byte by byte*/
    UART_Transmit( interface , (paket->address<<4) | (paket->byte_count<<1) );

    /*transmission of the data*/
    uint8_t i = 0;
    while ( i < (uint8_t)paket->byte_count ){
        UART_Transmit( interface,  (uint8_t)paket->bytes[i]);
        i = i + 1;
    }
}

uint8_t UART_Receive(uint8_t interface)
{
    /* This function is the basic receiver function who returns UDRn data to caller */
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


struct data_packet DATA_Receive( uint8_t interface )
{	 
	/*this function receives struct byte by byte*/
	static struct data_packet ReceivedPaket;
    //create an instance of a new paket to return once called by the ISR

    //receive the first byte that contain all the info we need for receive the rest
    uint8_t temp = UART_Receive( interface );
    ReceivedPaket.address = ( ( temp >> 4 ) & 0x0F );
    ReceivedPaket.byte_count = ( ( temp >> 1 ) & 0x07 );
	
	/*
	if (ReceivedPaket.byte_count != 2 || ReceivedPaket.address > 0xF || temp & 0x01)
	{
		return ReceivedPaket;
	}*/
	
    /*receive the rest of the data*/
    uint8_t i = 0;
	
	if ( ReceivedPaket.byte_count == 1 )
	{
		ReceivedPaket.bytes[0] = UART_Receive( interface );
	}
	else
	{
		for( i = 0; i < ReceivedPaket.byte_count; i++)
		{
			//receive the byte and add it to the struct by index from count
			ReceivedPaket.bytes[i] = UART_Receive( interface );
		}
		//return the now hopefully correct struct
		}
	return ReceivedPaket;
}


