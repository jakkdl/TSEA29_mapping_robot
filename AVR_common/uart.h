#ifndef UART_H
#define UART_H
#define UART_BAUD 207 //8MHz system clock

typedef struct
{
    /* raw header data */
    unsigned char  header;

    /* raw paket data after header */
    unsigned char datapaket[ 8 ];

} dataPaket; //datapaket strutc that contains header byte and data byte/bytes and relevants information

typedef union
{
    unsigned char b;
    struct //this party i compiler depended and should be checka and if possible force with flags
    {
        unsigned char b0 :1;
        unsigned char b1 :1;
        unsigned char b2 :1;
        unsigned char b3 :1;
        unsigned char b4 :1;
        unsigned char b5 :1;
        unsigned char b6 :1;
        unsigned char b7 :1;
    };
    
} byteUnion; //the unsiged char and the struct char the same memory thus allowing use to assces 1 bit at the time (in theory)

dataPaket dataPaket_Init( unsigned char header );
void UART_Init(uint8_t interface);
void UART_Transmit(uint8_t interface, unsigned char data);
unsigned char UART_Receive(uint8_t interface);
#endif
