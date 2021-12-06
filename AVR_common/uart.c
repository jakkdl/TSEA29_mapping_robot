#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#if __TEST__
#include "../AVR_testing/test.h"
#endif
#define UART_BAUD 8 //16MHz system clock 115.2k baud
//#define UART_BAUD 103 // 9.6k baud

/*
 *This code now work to make parity work we need to set setting on firefly so
 *8bit + 1 stop + even parity at 115,200 baud is the current setting that works i have adjusted to code
 */

void DATA_Transmit(struct ring_buffer* rb, struct data_packet *packet);
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
    init_ring_buffer(&g_uart_tx_0);
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

void DATA_Transmit(struct ring_buffer* rb, struct data_packet *packet)
{
    cli();
    // write header to the ring buffer
    write(rb, (packet->address<<4) | (packet->byte_count<<1) );

    // write data bytes
    for (uint8_t i = 0; i < packet->byte_count; ++i ){
        write(rb, packet->bytes[i]);
    }
    sei();
}

#if __UART_TX_0__
void Uart_Send_0(struct data_packet *packet)
{
#if !__TEST__
    DATA_Transmit(&g_uart_tx_0, packet);
    UCSR0A |= 1 << UDRE0;
#endif
}
#endif

#if __UART_TX_1__
void Uart_Send_1(struct data_packet *packet)
{
#if !__TEST__
    DATA_Transmit(&g_uart_tx_1, packet);
    UCSR1A |= 1 << UDRE1;
#endif
}
#endif

bool valid_header (uint8_t header)
{
    return ((!(header & 0x01)) &&
            ((header >> 4 == ADR_DEBUG)
             || ADR_DATA_PACKETS[header >> 4] == ((header >> 1) & 0x3)));
}

// called with interrupts disabled
void increment(struct ring_buffer* rb)
{
    if (++(rb->current) > rb->begin+RING_SZ-1)
    {
        rb->current = rb->begin;
    }
    --rb->length;
}

// called with interrupts disabled
// writes data to current+length
void write(struct ring_buffer* rb, uint8_t data)
{
    volatile uint8_t* last = rb->current + rb->length;
    if (last > rb->begin+RING_SZ-1)
    {
        last -= RING_SZ;
    }
    *last = data;
    if (++rb->length > RING_SZ)
    {
        rb->length = 0;
    }
}

bool Uart_Receive(struct ring_buffer* rb, struct data_packet *packet)
{
    cli();
    if (!rb->length)
    {
        sei();
        return false;
    }
    if (packet->bytes_read == 0xFF)
    {
        //while current is not a valid header, throw it away
        while (rb->length && !valid_header(*(rb->current)))
        {
            increment(rb);
        }
        //if there's no remaining packets, return false
        if (!rb->length)
        {
            sei();
            return false;
        }
        packet->address = (*(rb->current) >> 4);
        packet->byte_count = (*(rb->current) >> 1) & 0x3;
        packet->bytes_read = 0;
        increment(rb);
    }
    while (rb->length && packet->bytes_read < packet->byte_count)
    {
        packet->bytes[packet->bytes_read] = *rb->current;
        increment(rb);
        ++packet->bytes_read;
    }
    sei();
    return packet->bytes_read == packet->byte_count;
}

#if __UART_RX_0__
bool Uart_Receive_0(struct data_packet *packet)
{
    return Uart_Receive(&g_uart_rx_0, packet);
}
#endif

#if __UART_RX_1__
bool Uart_Receive_1(struct data_packet *packet)
{
    return Uart_Receive(&g_uart_rx_1, packet);
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

#if !__TEST__
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
#endif

#if __TEST__
Test_test(Test, write_0)
{
    struct ring_buffer rb;
    init_ring_buffer(&rb);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin);
    Test_assertEquals(rb.length, 0);

    write(&rb, 0xEF);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin);
    Test_assertEquals(rb.length, 1);
    Test_assertEquals(*(rb.current), 0xEF);

    write(&rb, 0xBE);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin);
    Test_assertEquals(rb.length, 2);
    Test_assertEquals(*(rb.current), 0xEF);
    Test_assertEquals(*(rb.current+1), 0xBE);

    struct data_packet data;
    data.bytes_read = 0xFF;
    data.address = 0;
    data.byte_count = 0;

    // read invalid headers
    Test_assertEquals(Uart_Receive(&rb, &data), false);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin+2);
    Test_assertEquals(rb.length, 0);
    Test_assertEquals(data.bytes_read, 0xFF);
    Test_assertEquals(data.address, 0);
    Test_assertEquals(data.byte_count, 0);
}

Test_test(Test, write_overflow)
{
    struct ring_buffer rb;
    init_ring_buffer(&rb);
    rb.current = rb.begin+RING_SZ-2;

    write(&rb, 0xC6);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin+RING_SZ-2);
    Test_assertEquals(rb.length, 1);
    Test_assertEquals(*(rb.current), 0xC6);

    write(&rb, 0xAB);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin+RING_SZ-2);
    Test_assertEquals(rb.length, 2);
    Test_assertEquals(*(rb.current+1), 0xAB);

    write(&rb, 0xAC);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin+RING_SZ-2);
    Test_assertEquals(rb.length, 3);
    Test_assertEquals(*(rb.begin), 0xAC);

    write(&rb, 0xAD);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin+RING_SZ-2);
    Test_assertEquals(rb.length, 4);
    Test_assertEquals(*(rb.begin+1), 0xAD);

    struct data_packet data;
    data.bytes_read = 0xFF;
    data.address = 0;
    data.byte_count = 0;

    Test_assertEquals(Uart_Receive(&rb, &data), true);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin+2);
    Test_assertEquals(rb.length, 0);
    Test_assertEquals(data.bytes_read, 0x3);
    Test_assertEquals(data.address, 0xc);
    Test_assertEquals(data.byte_count, 3);
    Test_assertEquals(data.bytes[0], 0xAB);
    Test_assertEquals(data.bytes[1], 0xAC);
    Test_assertEquals(data.bytes[2], 0xAD);
}

Test_test(Test, valid_header)
{
    Test_assertEquals(valid_header(0x14), true);
}

Test_test(Test, read)
{
    struct ring_buffer rb;
    init_ring_buffer(&rb);
    struct data_packet data;
    data.bytes_read = 0xFF;
    data.address = 0;
    data.byte_count = 0;


    write(&rb, 0x14);
    write(&rb, 0xFF);
    write(&rb, 0x00);

    Test_assertEquals(Uart_Receive(&rb, &data), true);
    Test_assertEquals(data.address, 1);
    Test_assertEquals(data.byte_count, 2);
    Test_assertEquals(data.bytes_read, 2);
    Test_assertEquals(data.bytes[0], 0xFF);
    Test_assertEquals(data.bytes[1], 0x00);
    Test_assertEquals(rb.length, 0);
    Test_assertEquals((uint16_t)rb.current, (uint16_t)rb.begin+3);

    data.bytes_read = 0xFF;

    write(&rb, 0x14);
    Test_assertEquals(rb.length, 1);
    Test_assertEquals(*(rb.current), 0x14);

    Test_assertEquals(Uart_Receive(&rb, &data), false);
    Test_assertEquals(data.address, 1);
    Test_assertEquals(data.byte_count, 2);
    Test_assertEquals(data.bytes_read, 0);

    write(&rb, 0xFF);
    Test_assertEquals(Uart_Receive(&rb, &data), false);
    Test_assertEquals(data.address, 1);
    Test_assertEquals(data.byte_count, 2);
    Test_assertEquals(data.bytes_read, 1);
    Test_assertEquals(data.bytes[0], 0xFF);

    write(&rb, 0x00);
    Test_assertEquals(Uart_Receive(&rb, &data), true);
    Test_assertEquals(data.address, 1);
    Test_assertEquals(data.byte_count, 2);
    Test_assertEquals(data.bytes_read, 2);
    Test_assertEquals(data.bytes[0], 0xFF);
    Test_assertEquals(data.bytes[1], 0x00);
}
#endif //__TEST__
