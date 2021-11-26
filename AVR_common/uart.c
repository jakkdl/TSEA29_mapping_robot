

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#define UART_BAUD 103 //16MHz system clock

#if __NAVIGATION_UNIT__
#include "../Navigation_unit/nav_sensor_loop.h"
#include "../Navigation_unit/nav_unit_com_interrupt_logic.h"
#endif

/*
 *TODO on hardware
 */



//intrups for tx should not be used as the restart the atmega for now
void UART_Init(uint8_t interface)
{
	/*this function set the correct bits in the various regs current baud is 9600 at clock at 16Mhz */
    if (interface == 0)
    {
        /* Set baud rate see page 174 in avr documentation*/
        // set upper part
        UBRR0H = (uint8_t)(UART_BAUD>>8);
        // set lower part
        UBRR0L = (uint8_t)UART_BAUD;

        /* Enable receiver and transmitter and tx and rx intrupts */
        UCSR0B = (1<<RXCIE0) | (0<<TXCIE0) |(1<<RXEN0)|(1<<TXEN0);

        /* Set frame format: 8data, 1stop bit, 1 parity bit */
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
        UCSR1B = (1<<RXCIE1) | (0<<TXCIE1) | (1<<RXEN1)| (1<<TXEN1);

        /* Set frame format: 8data, 1stop bit, 1 parity bit */
		UCSR1C =  (0<<USBS1) | (1<<UPM11) | (3<<UCSZ10);
    }
    /* 0_0_1_1_0_1_1_0*/
}


//intrups for tx should not be used as the restart the atmega for now
void UART_Transmit(uint8_t interface, uint8_t data )
{
	/*this funktion is the basic uart send funktion that puts data in the UDRn atmega does the rest*/
    if (interface == 0)
    {
        /* Wait for empty transmit buffer */
        while ( !( UCSR0A & (1<<UDRE0)) )
            ;
        /* Put data into buffer, sends the data */
        UDR0 = data;
		/*
		while( !( UCSR0A & (1 << TXC0) ))
		;*/
    }
    else
    {
        /* Wait for empty transmit buffer */
        while ( !( UCSR1A & (1<<UDRE1)) )
            ;
        /* Put data into buffer, sends the data */
        UDR1 = data;
		/*
		while( !( UCSR1A & (1 << TXC1) ))
		;*/
    }
}


//add interrupts to the transmit part of the UART transmit also this part has not been tested
void DATA_Transmit(uint8_t interface, struct data_packet *paket)
{
	/*This funktion sends a struct byte by byte*/
	
	cli(); //disable interrupts
    UART_Transmit( interface , (paket->address<<4) | (paket->byte_count<<1) );

    /*transmission of the data*/
    uint8_t i = 0;
    while ( i < (uint8_t)paket->byte_count ){
        UART_Transmit( interface,  (uint8_t)paket->bytes[i]);
        i = i + 1;
    }
	sei(); //re enable interrupts
}

uint8_t UART_Receive(uint8_t interface)
{
    /* This funktion is the basic reciver funktion who returns UDRn data to caller */
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
	/*this funktion recives struct byte by byte*/
	
    //creat an instance of a new paket to return once called by the ISR
    struct data_packet ReceivedPaket;

    //receive the first byte that contain all the info we need for receive the rest
    uint8_t temp = UART_Receive( interface );
	
	/*this part reverse the bit order so we get back correct info after serial tx*/
	uint8_t header = ( (temp>>4) & 0xF0) | ( ( temp << 4 ) & 0x0F );

    ReceivedPaket.address = ( ( header >> 4 ) & 0x0F );
    ReceivedPaket.byte_count = ( ( header >> 2 ) & 0x07 );

    /*receive the rest of the data*/
    uint8_t i = 0;
	
	//for loop implemetion as to not get stuck in case of miss math of packets
    for( i = 0; i < ReceivedPaket.byte_count; ++i)
	{
        /* Wait for data to be received */
        /* do we wait 2x the time here or not? I asume the check always passes
         * in the other function making that check redundant */	
        
		//this part reverse the bit order so we get back correct info after serial tx
		uint8_t temp = UART_Receive( interface );
        uint8_t CurrentRecivedPaket = ( (temp>>4) & 0xF0) | ( ( temp << 4 ) & 0x0F );
		
		//set no correct byte to a byte in the struct
        ReceivedPaket.bytes[i] = CurrentRecivedPaket;
		if ( interface == 0){
			while ( !(UCSR0A & (1<<RXC0)) )
			;
		}
		else
		{
			while ( !(UCSR1A & (1<<RXC1)) )
			;
		}
    }
	//return the now hopefully correct struct
    return ReceivedPaket;
}

#if __NAVIGATION_UNIT__
ISR( USART0_RX_vect )
{
    cli(); //disable interrupts
    struct data_packet received = DATA_Receive(0);
    sei(); //re enable interrupts
    handle_sensor_data(&received);
}
#endif

#if __NAVIGATION_UNIT__
ISR( USART1_RX_vect )
{
    cli(); //disable interrupts
    struct data_packet received = DATA_Receive(1);
    sei(); //re enable interrupts
    communication_unit_interrupt(&received);
}
#endif
