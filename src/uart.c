#include "./uart.h"

static FILE STDIO_BUFFER = FDEV_SETUP_STREAM(stdio_putchar, stdio_getchar, _FDEV_SETUP_RW);

// Based on ATMega328 datasheet p.184, Sending Frames with 5 to 8 Data Bit
int stdio_putchar(char c, FILE *stream)
{
    if (c == '\n')
        stdio_putchar('\r', stream);
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
    return 0;
}

// Based on ATMega328 datasheet p.187, Receiving Frames with 5 to 8 Data Bits
int stdio_getchar(FILE *stream)
{
    int UDR_Buff;
    while (!(UCSR0A & (1 << RXC0)))
        ;
    UDR_Buff = UDR0;
    stdio_putchar(UDR_Buff, stream);
    return UDR_Buff;
}

void NANO_stdio_set(void)
{
    unsigned int ubrr_data;

    // Asynchronous Normal Mode
    ubrr_data = F_CPU / 16 / DEFAULTUARTBAUD - 1;
    UBRR0H = (unsigned char)(ubrr_data >> 8);
    UBRR0L = (unsigned char)ubrr_data;

    // UCSR0B
    // Rx Enable, Tx Enable
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);

    // UCSRnC
    // Character Size : 8-bit
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);

    stdout = &STDIO_BUFFER;
    stdin = &STDIO_BUFFER;
}
