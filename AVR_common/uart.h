#ifndef UART_H
#define UART_H
#define UART_BAUD 207 //8MHz system clock

void UART_Init(uint8_t interface);
void UART_Transmit(uint8_t interface, unsigned char data);
unsigned char UART_Receive(uint8_t interface);
#endif
