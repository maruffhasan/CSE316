#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define BAUD 9600UL
#define UBRR_VALUE ((F_CPU / (8UL * BAUD)) - 1)

void UART_Init(void)
{
    uint16_t ubrr = UBRR_VALUE;

    // Double-speed UART
    UCSRA = (1 << U2X);

    // Baud rate
    UBRRH = (uint8_t)(ubrr >> 8);
    UBRRL = (uint8_t)ubrr;

    // Enable transmitter
    UCSRB = (1 << TXEN);

    // 8 data bits, 1 stop bit, no parity
    UCSRC =
        (1 << URSEL) |
        (1 << UCSZ1) |
        (1 << UCSZ0);
}

void UART_SendChar(char c)
{
    while (!(UCSRA & (1 << UDRE)));

    UDR = c;
}

void UART_SendString(const char *s)
{
    while (*s)
    {
        UART_SendChar(*s);
        s++;
    }
}

int main(void)
{
    UART_Init();

    while (1)
    {
        UART_SendString("HELLO FROM ATMEGA32A\r\n");

        _delay_ms(1000);
    }
}