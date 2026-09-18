#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

#define BAUD 9600UL
#define UBRR_VALUE ((F_CPU / (8UL * BAUD)) - 1)

// 64-point Sine wave lookup table (amplitude 127)
const int8_t Sinewave[64] = {
    0, 12, 25, 37, 49, 60, 71, 81, 90, 98, 106, 112, 117, 122, 125, 126, 
    127, 126, 125, 122, 117, 112, 106, 98, 90, 81, 71, 60, 49, 37, 25, 12, 
    0, -12, -25, -37, -49, -60, -71, -81, -90, -98, -106, -112, -117, -122, -125, -126, 
    -127, -126, -125, -122, -117, -112, -106, -98, -90, -81, -71, -60, -49, -37, -25, -12
};

void UART_Init(void) {
    uint16_t ubrr = UBRR_VALUE;
    UCSRA = (1 << U2X);
    UBRRH = (uint8_t)(ubrr >> 8);
    UBRRL = (uint8_t)ubrr;
    UCSRB = (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void UART_SendChar(char c) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = c;
}

void UART_SendString(const char *s) {
    while (*s) { UART_SendChar(*s++); }
}

void UART_SendNum(uint8_t num) {
    char buf[4];
    uint8_t i = 0;
    do {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    } while (num > 0);
    while (i > 0) UART_SendChar(buf[--i]);
}

void ADC_Init() {
    // AVCC Ref, Left Adjust Result (ADCH holds 8-bit MSB)
    ADMUX = (1 << REFS0) | (1 << ADLAR);
    // ADC Enable, Prescaler = 16 (for 62.5kHz ADC clock at 1MHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2);
}

int8_t ADC_Read() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    // MAX4466 idles at VCC/2. Subtract 128 to shift 0-255 into -128 to +127
    return (int8_t)(ADCH - 128); 
}

void compute_fft(int8_t fr[], int8_t fi[]) {
    uint8_t i, j, k, m, step, l;
    int16_t tr, ti;

    // Bit reversal
    j = 0;
    for (i = 0; i < 63; i++) {
        if (i < j) {
            tr = fr[j]; fr[j] = fr[i]; fr[i] = tr;
        }
        k = 32;
        while (k <= j) { j -= k; k >>= 1; }
        j += k;
    }

    // Radix-2 DIT FFT
    for (l = 1; l < 64; l = step) {
        step = l << 1;
        for (m = 0; m < l; m++) {
            uint8_t w_idx = (m * 32) / l;
            int8_t wr = Sinewave[(w_idx + 16) & 63]; // cos
            int8_t wi = -Sinewave[w_idx & 63];       // -sin

            for (i = m; i < 64; i += step) {
                j = i + l;
                tr = ((int16_t)wr * fr[j] - (int16_t)wi * fi[j]) >> 7;
                ti = ((int16_t)wr * fi[j] + (int16_t)wi * fr[j]) >> 7;
                
                fr[j] = fr[i] - tr;
                fi[j] = fi[i] - ti;
                fr[i] += tr;
                fi[i] += ti;
            }
        }
    }
}

int main(void) {
    int8_t real[64];
    int8_t imag[64];

    UART_Init();
    ADC_Init();

    while (1) {
        // 1. Sample 64 points
        for (uint8_t i = 0; i < 64; i++) {
            real[i] = ADC_Read();
            imag[i] = 0;
            _delay_us(100); // Determines max frequency range (~10kHz rate)
        }

        // 2. Perform FFT
        compute_fft(real, imag);

        // 3. Calculate magnitudes and print to UART
        // Bins 1 to 31 represent usable frequencies (0 is DC offset)
        for (uint8_t i = 1; i < 32; i++) {
            // Using abs(real)+abs(imag) to save costly sqrt() at 1MHz
            uint8_t magnitude = abs(real[i]) + abs(imag[i]); 
            
            UART_SendNum(magnitude);
            if (i < 31) UART_SendChar(',');
        }
        UART_SendString("\r\n");
    }
}