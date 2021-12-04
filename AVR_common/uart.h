#include <stdint.h>
#include <stdbool.h>
#ifndef UART_H_
#define UART_H_
#include "robot.h"

extern volatile uint8_t** receive_buffer;
extern volatile uint8_t** send_buffer;

void UART_Init(uint8_t interface, bool rx, bool tx);
void UART_Transmit(uint8_t interface, uint8_t data);
void DATA_Transmit(uint8_t interface, struct data_packet *paket);
uint8_t UART_Receive(uint8_t interface);
struct data_packet DATA_Receive( uint8_t interface );

#endif /* UART_H_ */
