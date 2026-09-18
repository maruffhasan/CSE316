#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================
// SETTINGS
// ============================================================

#define FFT_SIZE       128
#define SAMPLE_RATE    8000UL
#define UART_BAUD      38400UL

// Mapping limits.
// These are RELATIVE FFT dB values, NOT dB SPL.

#define FLOOR_DB       20.0f
#define CEILING_DB     65.0f


// ============================================================
// 10 LOGARITHMIC FREQUENCY BANDS
// 250 Hz -> 4000 Hz
// ============================================================

const float bandEdges[11] = {
    250.0f,
    329.9f,
    435.3f,
    574.3f,
    757.9f,
    1000.0f,
    1319.5f,
    1741.1f,
    2297.4f,
    3031.4f,
    4000.0f
};


// ============================================================
// FFT ARRAYS
// ============================================================

float realData[FFT_SIZE];
float imagData[FFT_SIZE];

volatile uint16_t audioBuffer[FFT_SIZE];

volatile uint8_t sampleIndex = 0;
volatile uint8_t frameReady = 0;

float bandLevel[10];


// ============================================================
// UART
// ============================================================

void UART_Init(void)
{
    uint16_t ubrr;

    ubrr =
        (uint16_t)(
            (F_CPU / (16UL * UART_BAUD)) - 1
        );

    UBRRH =
        (uint8_t)(ubrr >> 8);

    UBRRL =
        (uint8_t)ubrr;

    // Enable transmitter
    UCSRB =
        (1 << TXEN);

    // 8 data bits, 1 stop bit
    UCSRC =
        (1 << URSEL) |
        (1 << UCSZ1) |
        (1 << UCSZ0);
}


void UART_SendByte(uint8_t data)
{
    while (!(UCSRA & (1 << UDRE)));

    UDR = data;
}


void UART_SendString(const char *str)
{
    while (*str)
    {
        UART_SendByte(
            (uint8_t)*str
        );

        str++;
    }
}


void UART_SendUInt8(uint8_t value)
{
    char buffer[4];

    uint8_t i = 0;

    if (value >= 100)
        buffer[i++] =
            '0' + value / 100;

    if (value >= 10)
        buffer[i++] =
            '0' + (value / 10) % 10;

    buffer[i++] =
        '0' + value % 10;

    buffer[i] = '\0';

    UART_SendString(buffer);
}


// ============================================================
// ADC INITIALIZATION
//
// ADC0
// AVCC reference
// ADC clock = 16 MHz / 128 = 125 kHz
// ============================================================

void ADC_Init(void)
{
    ADMUX =
        (1 << REFS0);      // AVCC reference
                           // ADC0 selected

    ADCSRA =
        (1 << ADEN)  |     // ADC enable
        (1 << ADIE)  |     // ADC interrupt
        (1 << ADPS2) |
        (1 << ADPS1) |
        (1 << ADPS0);      // /128
}


// ============================================================
// TIMER1
//
// 16 MHz / 8000 Hz = 2000 clocks
//
// CTC:
// OCR1A = 1999
// ============================================================

void Timer1_Init(void)
{
    TCCR1A = 0;

    TCCR1B =
        (1 << WGM12) |     // CTC
        (1 << CS10);       // no prescaler

    OCR1A = 1999;

    TIMSK =
        (1 << OCIE1A);
}


// ============================================================
// TIMER1 ISR
//
// Generate exact 8 kHz sampling.
// ============================================================

ISR(TIMER1_COMPA_vect)
{
    if (!frameReady)
    {
        ADCSRA |=
            (1 << ADSC);
    }
}


// ============================================================
// ADC ISR
// ============================================================

ISR(ADC_vect)
{
    uint16_t value;

    value = ADC;

    if (frameReady)
        return;

    audioBuffer[sampleIndex] =
        value;

    sampleIndex++;

    if (sampleIndex >= FFT_SIZE)
    {
        sampleIndex = 0;
        frameReady = 1;
    }
}


// ============================================================
// COPY FRAME
// ============================================================

void copyFrame(void)
{
    uint8_t i;

    cli();

    for (i = 0; i < FFT_SIZE; i++)
    {
        realData[i] =
            (float)audioBuffer[i];

        imagData[i] =
            0.0f;
    }

    frameReady = 0;

    sei();
}


// ============================================================
// REMOVE DC OFFSET
// ============================================================

void removeDC(void)
{
    uint8_t i;

    float sum = 0.0f;

    for (i = 0; i < FFT_SIZE; i++)
    {
        sum += realData[i];
    }

    float mean =
        sum / FFT_SIZE;

    for (i = 0; i < FFT_SIZE; i++)
    {
        realData[i] -= mean;
    }
}


// ============================================================
// HANN WINDOW
// ============================================================

void applyWindow(void)
{
    uint8_t i;

    for (i = 0; i < FFT_SIZE; i++)
    {
        float window =
            0.5f *
            (
                1.0f -
                cos(
                    2.0f * M_PI *
                    i /
                    (FFT_SIZE - 1)
                )
            );

        realData[i] *=
            window;
    }
}


// ============================================================
// BIT REVERSAL
// ============================================================

void bitReverse(void)
{
    uint8_t i;
    uint8_t j = 0;

    for (i = 1; i < FFT_SIZE; i++)
    {
        uint8_t bit =
            FFT_SIZE >> 1;

        while (j & bit)
        {
            j ^= bit;
            bit >>= 1;
        }

        j ^= bit;

        if (i < j)
        {
            float temp;

            temp =
                realData[i];

            realData[i] =
                realData[j];

            realData[j] =
                temp;


            temp =
                imagData[i];

            imagData[i] =
                imagData[j];

            imagData[j] =
                temp;
        }
    }
}


// ============================================================
// FFT
// ============================================================

void FFT(void)
{
    uint16_t len;

    bitReverse();

    for (
        len = 2;
        len <= FFT_SIZE;
        len <<= 1
    )
    {
        float angle =
            -2.0f * M_PI / len;

        float wRealStep =
            cos(angle);

        float wImagStep =
            sin(angle);

        uint16_t i;

        for (
            i = 0;
            i < FFT_SIZE;
            i += len
        )
        {
            float wReal = 1.0f;
            float wImag = 0.0f;

            uint16_t j;

            for (
                j = 0;
                j < len / 2;
                j++
            )
            {
                uint16_t u =
                    i + j;

                uint16_t v =
                    i + j +
                    len / 2;


                float vReal =
                    realData[v] *
                    wReal -
                    imagData[v] *
                    wImag;

                float vImag =
                    realData[v] *
                    wImag +
                    imagData[v] *
                    wReal;


                float uReal =
                    realData[u];

                float uImag =
                    imagData[u];


                realData[u] =
                    uReal + vReal;

                imagData[u] =
                    uImag + vImag;


                realData[v] =
                    uReal - vReal;

                imagData[v] =
                    uImag - vImag;


                float newWReal =
                    wReal *
                    wRealStep -
                    wImag *
                    wImagStep;

                float newWImag =
                    wReal *
                    wImagStep +
                    wImag *
                    wRealStep;

                wReal =
                    newWReal;

                wImag =
                    newWImag;
            }
        }
    }
}


// ============================================================
// CALCULATE 10 BANDS
// ============================================================

void calculateBands(void)
{
    uint8_t k;
    uint8_t b;

    for (b = 0; b < 10; b++)
    {
        bandLevel[b] =
            0.0f;
    }


    // Positive-frequency bins
    //
    // Bin resolution:
    //
    // 8000 / 128 = 62.5 Hz

    for (
        k = 1;
        k <= FFT_SIZE / 2;
        k++
    )
    {
        float frequency =
            ((float)k *
             SAMPLE_RATE) /
            FFT_SIZE;


        if (frequency < 250.0f)
            continue;

        if (frequency > 4000.0f)
            continue;


        float magnitude =
            sqrt(
                realData[k] *
                realData[k] +
                imagData[k] *
                imagData[k]
            );


        for (b = 0; b < 10; b++)
        {
            if (
                frequency >=
                bandEdges[b] &&

                frequency <
                bandEdges[b + 1]
            )
            {
                bandLevel[b] +=
                    magnitude *
                    magnitude;

                break;
            }
        }
    }


    // Convert accumulated power to
    // relative logarithmic magnitude

    for (b = 0; b < 10; b++)
    {
        float magnitude =
            sqrt(
                bandLevel[b]
            );

        bandLevel[b] =
            20.0f *
            log10(
                magnitude + 1.0f
            );
    }
}


// ============================================================
// MAP TO 0...90
//
// 0  -> 0.00
// 90 -> 0.90
// ============================================================

uint8_t mapLevel(float db)
{
    float result;

    if (db <= FLOOR_DB)
        return 0;

    if (db >= CEILING_DB)
        return 90;


    result =
        (
            db -
            FLOOR_DB
        )
        /
        (
            CEILING_DB -
            FLOOR_DB
        );


    result *=
        90.0f;


    if (result < 0)
        result = 0;

    if (result > 90)
        result = 90;


    return
        (uint8_t)(
            result + 0.5f
        );
}


// ============================================================
// SEND ONE COMPLETE FRAME
//
// Format:
//
// S,12,25,31,45,67,90,71,50,22,8,E
//
// All values are 0...90.
// UNO converts value/100 -> 0.00...0.90
// ============================================================

void sendSpectrum(void)
{
    uint8_t i;
    uint8_t value;

    UART_SendString(
        "S,"
    );


    for (i = 0; i < 10; i++)
    {
        value =
            mapLevel(
                bandLevel[i]
            );

        UART_SendUInt8(
            value
        );

        if (i < 9)
        {
            UART_SendByte(',');
        }
    }


    UART_SendString(
        ",E\r\n"
    );
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    UART_Init();

    ADC_Init();

    Timer1_Init();

    sei();


    UART_SendString(
        "ATMEGA32A_SPECTRUM_READY\r\n"
    );


    while (1)
    {
        if (frameReady)
        {
            copyFrame();

            removeDC();

            applyWindow();

            FFT();

            calculateBands();

            sendSpectrum();
        }
    }


    return 0;
}
	
