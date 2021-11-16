#include <stdint.h>
#ifndef UART_H_
#define UART_H_

typedef struct
{
	/* raw header data */
	uint8_t  header;

	/* raw packet data after header */
	uint8_t datapaket[ 7 ];

} dataPaket; //datapaket strutc that contains header byte and data byte/bytes and relevant information

typedef union
{
	unsigned char b;
	struct //this party i compiler depended and should be check and if possible force with flags
	{
		uint8_t b0 :1;
		uint8_t b1 :1;
		uint8_t b2 :1;
		uint8_t b3 :1;
		uint8_t b4 :1;
		uint8_t b5 :1;
		uint8_t b6 :1;
		uint8_t b7 :1;
	};
	
} byteUnion; //the unsigned char and the struct char the same memory thus allowing use to accesses 1 bit at the time (in theory)

dataPaket dataPaket_Init( uint8_t header );
void UART_Init(uint8_t interface);
void UART_Transmit(uint8_t interface, uint8_t data);
uint8_t UART_Receive(uint8_t interface);
dataPaket DATA_Receive( uint8_t interface );
void parityError( uint8_t interface, dataPaket paket );

#endif /* UART_H_ */
