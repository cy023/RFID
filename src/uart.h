#ifndef UART_H
#define UART_H

#define F_CPU 16000000UL
#define DEFAULTUARTBAUD 9600

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int stdio_putchar(char c, FILE *stream);
int stdio_getchar(FILE *stream);
void NANO_stdio_set(void);

#endif