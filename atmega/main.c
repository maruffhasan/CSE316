#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

// =====================================================
// UART INITIALIZATION
// ATmega32 clock = 1 MHz
// Baud rate       = 9600
// UART mode       = 8-N-1
// Double speed    = enabled
// UBRR            = 12
// =====================================================

void UART_Init(void)
{
    // 9600 baud @ 1 MHz using U2X
    UBRRH = 0;
    UBRRL = 12;

    // Double-speed mode
    UCSRA |= (1 << U2X);

    // Enable transmitter
    UCSRB = (1 << TXEN);

    // 8-bit data
    // 1 stop bit
    // No parity
    UCSRC = (1 << URSEL) |
            (1 << UCSZ1) |
            (1 << UCSZ0);
}


// =====================================================
// SEND ONE CHARACTER
// =====================================================

void UART_SendChar(char c)
{
    // Wait until transmit buffer is empty
    while (!(UCSRA & (1 << UDRE)));

    // Put character into UART data register
    UDR = c;
}


// =====================================================
// SEND STRING
// =====================================================

void UART_SendString(const char *str)
{
    while (*str)
    {
        UART_SendChar(*str);
        str++;
    }
}


// =====================================================
// MAIN
// =====================================================

int main(void)
{
    UART_Init();

    while (1)
    {
        UART_SendString("HELLO FROM ATMEGA32\r\n");

        _delay_ms(1000);
    }
}