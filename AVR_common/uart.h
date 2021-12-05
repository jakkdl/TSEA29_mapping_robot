#ifndef UART_H_
#define UART_H_
#include <stdint.h>
#include <stdbool.h>
#include "robot.h"

#define RING_SZ 32
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

#endif /* UART_H_ */
