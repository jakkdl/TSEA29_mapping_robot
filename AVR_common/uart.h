#ifndef UART_H_
#define UART_H_
#include <stdint.h>
#include <stdbool.h>
#include "robot.h"

#define RING_SZ 128
struct ring_buffer
{
    volatile uint8_t begin[RING_SZ];
    volatile uint8_t* current;
    uint8_t length;
};

#if __UART_RX_0__
bool Uart_Receive_0(struct data_packet *paket);
#endif

#if __UART_TX_0__
void Uart_Send_0(struct data_packet *paket);
#endif

#if __UART_RX_1__
bool Uart_Receive_1(struct data_packet *paket);
#endif

#if __UART_TX_1__
void Uart_Send_1(struct data_packet *paket);
#endif

void Uart_Init(void);

#if __TEST__
extern struct ring_buffer g_uart_rx_0;
extern struct ring_buffer g_uart_tx_0;
extern struct ring_buffer g_uart_rx_1;
#if __UART_TX_1__
extern struct ring_buffer g_uart_tx_1;
#endif
#endif
#endif /* UART_H_ */
