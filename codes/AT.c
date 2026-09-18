#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

#define BAUD 9600UL
#define UBRR_VALUE ((F_CPU / (8UL * BAUD)) - 1)

#define FFT_SIZE 128
#define SAMPLE_RATE 8000UL

// 64-point sine wave lookup table
// Values represent sin(2*pi*n/64) * 127
const int8_t Sinewave[64] = {
     0,  12,  25,  37,  49,  60,  71,  81,
    90,  98, 106, 112, 117, 122, 125, 126,
   127, 126, 125, 122, 117, 112, 106,  98,
    90,  81,  71,  60,  49,  37,  25,  12,
     0, -12, -25, -37, -49, -60, -71, -81,
   -90, -98,-106,-112,-117,-122,-125,-126,
  -127,-126,-125,-122,-117,-112,-106, -98,
   -90, -81, -71, -60, -49, -37, -25, -12
};


// ============================================================
// UART
// ============================================================

void UART_Init(void)
{
    uint16_t ubrr = UBRR_VALUE;

    UCSRA = (1 << U2X);

    UBRRH = (uint8_t)(ubrr >> 8);
    UBRRL = (uint8_t)ubrr;

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


void UART_SendNum(uint16_t num)
{
    char buf[6];
    uint8_t i = 0;

    do
    {
        buf[i++] = (num % 10) + '0';
        num /= 10;

    } while (num > 0);

    while (i > 0)
    {
        UART_SendChar(buf[--i]);
    }
}


void UART_SendString(const char *s)
{
    while (*s)
    {
        UART_SendChar(*s++);
    }
}


// ============================================================
// ADC
// ============================================================

void ADC_Init(void)
{
    /*
     * AVCC reference
     * Left-adjust result
     * ADC channel = ADC0
     */
    ADMUX = (1 << REFS0) |
            (1 << ADLAR);

    /*
     * ADC enabled
     *
     * Prescaler = 8
     *
     * F_CPU = 1 MHz
     * ADC clock = 1 MHz / 8 = 125 kHz
     */
    ADCSRA = (1 << ADEN) |
             (1 << ADPS1) |
             (1 << ADPS0);
}


uint8_t ADC_Read(void)
{
    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC));

    return ADCH;
}


// ============================================================
// FFT
// ============================================================

void compute_fft(int8_t real[], int8_t imag[])
{
    uint8_t i, j, k;
    uint8_t len, half;
    int16_t tr, ti;

    /*
     * Bit reversal
     */

    j = 0;

    for (i = 0; i < FFT_SIZE - 1; i++)
    {
        if (i < j)
        {
            int8_t temp;

            temp = real[i];
            real[i] = real[j];
            real[j] = temp;

            temp = imag[i];
            imag[i] = imag[j];
            imag[j] = temp;
        }

        k = FFT_SIZE >> 1;

        while (k <= j)
        {
            j -= k;
            k >>= 1;
        }

        j += k;
    }


    /*
     * Radix-2 FFT
     */

    for (len = 2; len <= FFT_SIZE; len <<= 1)
    {
        half = len >> 1;

        for (j = 0; j < half; j++)
        {
            /*
             * Generate twiddle factor.
             *
             * FFT_SIZE = 128
             * Sine table = 64 points
             *
             * angle index = j * 64 / len
             */

            uint8_t w_idx;

            w_idx = (uint8_t)(((uint16_t)j * 64) / len);

            /*
             * cos(theta) = sin(theta + 90 degrees)
             */

            int8_t wr =
                Sinewave[(w_idx + 16) & 63];

            /*
             * -sin(theta)
             */

            int8_t wi =
                -Sinewave[w_idx & 63];


            for (i = j; i < FFT_SIZE; i += len)
            {
                k = i + half;

                /*
                 * Complex multiplication
                 *
                 * (wr + j*wi) * (real[k] + j*imag[k])
                 */

                tr =
                    ((int16_t)wr * real[k]
                    -
                    (int16_t)wi * imag[k]) >> 7;

                ti =
                    ((int16_t)wr * imag[k]
                    +
                    (int16_t)wi * real[k]) >> 7;


                /*
                 * Butterfly
                 */

                int16_t r1 = real[i];
                int16_t i1 = imag[i];

                real[k] = r1 - tr;
                imag[k] = i1 - ti;

                real[i] = r1 + tr;
                imag[i] = i1 + ti;
            }
        }
    }
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    int8_t real[FFT_SIZE];
    int8_t imag[FFT_SIZE];

    uint8_t i;

    UART_Init();
    ADC_Init();


    while (1)
    {
        // ====================================================
        // 1. CAPTURE 128 SAMPLES
        // ====================================================

        for (i = 0; i < FFT_SIZE; i++)
        {
            /*
             * ADC = 0 ... 255
             *
             * Microphone signal is centered around ~128
             *
             * Convert to:
             *
             * -128 ... +127
             */

            real[i] =
                (int8_t)((int16_t)ADC_Read() - 128);

            imag[i] = 0;


            /*
             * 8 kHz sampling
             *
             * Period = 125 us
             *
             * ADC conversion itself takes most of this time.
             *
             * Delay is used to make the sampling interval
             * approximately 125 us.
             */

            _delay_us(125);
        }


        // ====================================================
        // 2. FFT
        // ====================================================

        compute_fft(real, imag);


        // ====================================================
        // 3. CALCULATE 10 LOGARITHMIC BANDS
        // ====================================================

        /*
         * Frequency resolution:
         *
         * Fs / N
         *
         * = 8000 / 128
         *
         * = 62.5 Hz/bin
         *
         *
         * FFT bin frequency:
         *
         * frequency = bin * 62.5 Hz
         *
         *
         * Logarithmic boundaries:
         *
         * 250
         * 330
         * 435
         * 574
         * 758
         * 1000
         * 1319
         * 1741
         * 2297
         * 3029
         * 4000 Hz
         */


        const uint8_t band_start[10] =
        {
            4,    // 250 Hz
            5,    // 312.5 Hz
            7,    // 437.5 Hz
            9,    // 562.5 Hz
            12,   // 750 Hz
            16,   // 1000 Hz
            21,   // 1312.5 Hz
            28,   // 1750 Hz
            37,   // 2312.5 Hz
            49    // 3062.5 Hz
        };


        const uint8_t band_end[10] =
        {
            5,    // ~330 Hz
            7,    // ~435 Hz
            9,    // ~574 Hz
            12,   // ~758 Hz
            16,   // ~1000 Hz
            21,   // ~1319 Hz
            28,   // ~1741 Hz
            37,   // ~2297 Hz
            49,   // ~3029 Hz
            64    // 4000 Hz
        };


        /*
         * Calculate one amplitude value for
         * each frequency band.
         */

        for (uint8_t band = 0; band < 10; band++)
        {
            uint16_t sum = 0;

            uint8_t start =
                band_start[band];

            uint8_t end =
                band_end[band];


            for (uint8_t bin = start;
                         bin < end;
                         bin++)
            {
                /*
                 * Approximate magnitude:
                 *
                 * |Real| + |Imag|
                 *
                 * Much cheaper than sqrt().
                 */

                uint8_t magnitude =
                    abs(real[bin]) +
                    abs(imag[bin]);


                sum += magnitude;
            }


            /*
             * Average the magnitude
             * of all FFT bins in this band.
             */

            uint8_t number_of_bins =
                end - start;

            uint16_t amplitude =
                sum / number_of_bins;


            /*
             * Send result
             */

            UART_SendNum(amplitude);


            if (band < 9)
            {
                UART_SendChar(',');
            }
        }


        // End of one FFT frame

        UART_SendString("\r\n");
    }
}