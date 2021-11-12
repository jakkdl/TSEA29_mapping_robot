#include <avr/io.h>
#include "uart.h"


/*
 *none of the below code has been tested as of yet need to be tested nor complied and
 *it is to be viewed as pusod code more or less untill compilation and testing
 */

void UART_Init(uint8_t interface)
{
    if (interface == 0)
    {
    /* Set baud rate see page 174 in avr doccumentationc*/
        // set upper part
        UBRR0H = (unsigned char)(UART_BAUD>>8);
        // set lower part
        UBRR0L = (unsigned char)UART_BAUD;

        /* Enable receiver and transmitter */
        UCSR0B = (1<<RXEN0)|(1<<TXEN0);

        /* Set frame format: 8data, 1stop bit, 1 parity bit */
        UCSR0C = (3<<UCSZ00);
    }
    else
    {
        /* Set baud rate see page 174 in avr doccumentationc*/
        // set upper part
        UBRR1H = (unsigned char)(UART_BAUD>>8);
        // set lower part
        UBRR1L = (unsigned char)UART_BAUD;

        /* Enable receiver and transmitter */
        UCSR1B = (1<<RXEN1)|(1<<TXEN1);

        /* Set frame format: 8data, 1stop bit, 1 parity bit */
        UCSR1C = (3<<UCSZ10);
    }
    /* 0_0_1_1_0_1_1_0*/
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

//add intrupts to the transmip part of the uart transmit also this part has not been tested
void DATA_Transmit(uint8_t interface, struct dataPaket *paket){
        
        /*transmits the header paket*/
        UART_Transmit(interface, paket.header);

        /*transmists the rest of the data*/
        uint8_t i = 0;
        while ( i < 8){
            unsigned char paket = paket.databytes[i];
            /* Wait for empty transmit buffer */
             while ( !( UCSR1A & (1<<UDRE1)) )
            ;
            UART_Transmit(interface, paket);
            i = i + 1;
        }
}

unsigned char UART_Receive(uint8_t interface){

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


//TODO add parity checkers and intrupts to this part also this has not been tested
dataPaket DATA_Receive( unti8_t interface ){
    dataPaket ReceivedPaket;
    ReceivedPaket.header =  UART_Receive(uint8_t interface);
     /*transmists the rest of the data*/
    uint8_t i = 0;
    while ( i < 8){
        /* Wait for data to be received */
        while ( !(UCSR1A & (1<<RXC1)) )
            ;
        unsigned char recivedpaket = ART_Receive(uint8_t interface);
        paket.datapaket[i] = recivedpaket;
    }

}


//I have no idea if this part will or works at all but something like this could be used check back when trying to complie in atmel
void parityError( uint8_t interface, struct dataPaket *paket ){
    byteUnion headerbyte = paket.header;
    dataPaket erroPaket;
    erroPaket.header = ( 7<<1 ) | ( 6<<1 ) | ( 5<<1 ) | ( 4<<1 ) | ( 3<<0 ) | ( 2<<0 ) | ( 1<<1 ) | ( 0 ); //correct bits need to be set here 
    erroPaket.datapaket[0] = ( 3<<headerbyte.b7 ) | (2<<headerbyte.b6) | (1<<headerbyte.b5) | ( headerbyte.b4 );
    DATA_Transmit( erroPaket, interface );
}
