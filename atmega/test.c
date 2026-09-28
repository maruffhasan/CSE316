#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

void UART_Init(void)
{
    UBRRH = 0;
    UBRRL = 12;

    UCSRA |= (1 << U2X);

    UCSRB = (1 << TXEN);

    UCSRC = (1 << URSEL) |
            (1 << UCSZ1) |
            (1 << UCSZ0);
}

void UART_SendChar(char c)
{
    while (!(UCSRA & (1 << UDRE)));

    UDR = c;
}

void UART_SendString(const char *str)
{
    while (*str)
        UART_SendChar(*str++);
}

int main(void)
{
    UART_Init();

    while (1)
    {
        UART_SendString("HELLO FROM ATMEGA32\r\n");
        _delay_ms(1000);
    }
}