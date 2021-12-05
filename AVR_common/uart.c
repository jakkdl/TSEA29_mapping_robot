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

void DATA_Transmit(struct ring_buffer* rb, struct data_packet *paket);
bool valid_header (uint8_t header);
void increment(struct ring_buffer* rb);
void write(struct ring_buffer* rb, uint8_t data);

#if __UART_RX_0__
struct ring_buffer g_uart_rx_0;
#endif
#if __UART_TX_0__
struct ring_buffer g_uart_tx_0;
#endif
#if __UART_RX_1__
struct ring_buffer g_uart_rx_1;
#endif
#if __UART_TX_1__
struct ring_buffer g_uart_tx_1;
#endif

void init_ring_buffer(struct ring_buffer* rb)
{
    rb->current = rb->begin;
    rb->length = 0;
}

void Uart_Init(void)
{
#if __UART_RX_0__ | __UART_TX_0__
    UBRR0H = (uint8_t)(UART_BAUD>>8);
    UBRR0L = (uint8_t)(UART_BAUD & 0xFFFF);
    UCSR0C = (0<<USBS0) |(1<<UPM01) | (3<<UCSZ00);
#endif
#if __UART_RX_1__ | __UART_TX_1__
    UBRR1H = (uint8_t)(UART_BAUD>>8);
    UBRR1L = (uint8_t)(UART_BAUD & 0xFFFF);
    UCSR1C = (0<<USBS1) |(1<<UPM11) | (3<<UCSZ10);
#endif
#if __UART_RX_0__
    UCSR0B |= (1<<RXEN0);
    init_ring_buffer(&g_uart_rx_0);
#endif
#if __UART_TX_0__
    UCSR0B |= (1<<TXEN0);
    init_ring_buffer(&g_uart_rx_0);
#endif
#if __UART_RX_1__
    UCSR1B |= (1<<RXEN1);
    init_ring_buffer(&g_uart_rx_1);
#endif
#if __UART_TX_1__
    UCSR1B |= (1<<TXEN1);
    init_ring_buffer(&g_uart_tx_1);
#endif
}

void DATA_Transmit(struct ring_buffer* rb, struct data_packet *paket)
{
    // write header to the ring buffer
    write(rb, (paket->address<<4) | (paket->byte_count<<1) );

    // write data bytes
    for (uint8_t i = 0; i < paket->byte_count; ++i ){
        write(rb, paket->bytes[i]);
    }
}

#if __UART_TX_0__
void Uart_Send_0(struct data_packet *paket)
{

    DATA_Transmit(&g_uart_rx_0, paket);
    UCSR0A |= 1 << UDRE0;
}
#endif

#if __UART_TX_0__
void Uart_Send_1(struct data_packet *paket)
{
    DATA_Transmit(&g_uart_rx_1, paket);
    UCSR1A |= 1 << UDRE1;
}
#endif

bool valid_header (uint8_t header)
{
    return ((!header & 0x01) &&
            (header >> 4 == ADR_DEBUG
             || ADR_DATA_PACKETS[header >> 4] == ((header >> 1) & 0x3)));
}

void increment(struct ring_buffer* rb)
{
    if (++(rb->current) > rb->begin+RING_SZ)
    {
        rb->current = rb->begin;
    }
}

void write(struct ring_buffer* rb, uint8_t data)
{
    volatile uint8_t* last = rb->current + rb->length;
    if (last > rb->begin+RING_SZ)
    {
        last -= RING_SZ;
    }
    *last = data;
    ++rb->length;
}


bool Uart_Receive(struct ring_buffer* rb, struct data_packet *paket)
{
    if (!rb->length)
    {
        return false;
    }
    //while current is not a valid header, throw it away
    while (!valid_header(*(rb->current)) && rb->length)
    {
        increment(rb);
    }
    //if there's no remaining packets, return false
    if (!rb->length)
    {
        return false;
    }
    //if current header is waiting for more bytes, return false
    if (rb->length < (*(rb->current) >> 4) + 1)
    {
        return false;
    }
    //else set struct
    paket->byte_count = (*(rb->current) >> 1) & 0x3;
    paket->address = (*(rb->current) >> 4);
    for (int8_t i = 0; i < paket->byte_count; ++i)
    {
        increment(rb);
        paket->bytes[i] = *rb->current;
    }
    return true;
}

#if __UART_RX_0__
bool Uart_Receive_0(struct data_packet *paket)
{
    return Uart_Receive(&g_uart_rx_0, paket);
}
#endif

#if __UART_RX_1__
bool Uart_Receive_1(struct data_packet *paket)
{
    return Uart_Receive(&g_uart_rx_1, paket);
}
#endif

#define UART_RECEIVE_FUNC(x) \
    write(&g_uart_rx_##x, UDR##x);

#define UART_SEND_FUNC(x) \
    if (g_uart_tx_ ## x.length == 0) \
    { \
        /* no more data to send */ \
        /* don't raise interrupt on empty buffer */ \
        UCSR ## x ## A &= ~(1 << UDRE ## x); \
    } \
    else \
    { \
        increment(&g_uart_tx_##x); \
        UDR##x = *(g_uart_tx_##x.current); \
    } \


#if __UART_RX_0__
ISR( USART0_RX_vect )
{
    UART_RECEIVE_FUNC(0)
}
#endif
#if __UART_TX_0__
ISR( USART0_UDRE_vect )
{
    UART_SEND_FUNC(0)
}
#endif
#if __UART_RX_1__
ISR( USART1_RX_vect )
{
    UART_RECEIVE_FUNC(1)
}
#endif
#if __UART_TX_1__
ISR ( USART1_UDRE_vect )
{
    UART_SEND_FUNC(1)
}
#endif
