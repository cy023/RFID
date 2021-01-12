#define F_CPU 16000000UL
#define LED_PIN 5

#include "uart.h"

int main()
{
    NANO_stdio_set();
    printf("hihi\n");
    DDRB |= (1 << LED_PIN);
    while (1)
    {
        PORTB ^= (1 << LED_PIN);
        _delay_ms(500);
    }
    return 0;
}