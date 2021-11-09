

void UART_Init( unsigned int baud )
{
    /* Set baud rate see page 174 in avr doccumentationc*/

    /* set uperpart of the baud rate */
    UBRRHn = (unsigned char)(baud>>8);
    /* set lower part of the baud rate */
    UBRRLn = (unsigned char)baud;

    /* Enable receiver and transmitter */
    UCSRnB = (1<<RXENn)|(1<<TXENn);

    /* Set frame format: 8data, 1stop bit, 1 even parity bit */
    UCSRnC = (3<<UCSZn0)|(2<<UPMn0);

    /* 0_0_1_1_0_1_1_0*/
}


void USART_Transmit( unsigned char data )
{
    /* Wait for empty transmit buffer */
    while ( !( UCSRnA & (1<<UDREn)) );

    /* Put data into buffer, sends the data */
    UDRn = data;
}



unsigned char USART_Receive( void )
{
    /* Wait for data to be received */
    while ( !(UCSRnA & (1<<RXCn)) )
    ;
    /* Get and return received data from buffer */
    return UDRn;
}