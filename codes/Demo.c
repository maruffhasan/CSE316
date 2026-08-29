#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "ILI9341.h"

// ILI9341 Color Definitions (RGB565 format)
#define ILI9341_BLACK   0x0000
#define ILI9341_WHITE   0xFFFF
#define ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_RED     0xF800

#define FFT_SIZE     32
#define FFT_STAGES   5
#define NUM_BARS     16

#define PEAK_HOLD    3

// --- Screen / layout geometry (320x240 landscape) ---
// Bars are drawn as NUM_BARS vertical columns spanning the screen width,
// anchored at the bottom and growing upward -- a standard spectrum-analyzer
// layout, rather than the previous stacked horizontal rows.
#define SCREEN_WIDTH   320
#define SCREEN_HEIGHT  240

#define HEADER_HEIGHT  16
#define COL_WIDTH      (SCREEN_WIDTH / NUM_BARS)
#define COL_GAP        2
#define BAR_THICK      (COL_WIDTH - COL_GAP)

#define BAR_Y_BASE     (SCREEN_HEIGHT - 2)
#define BAR_MAX_LEN    (SCREEN_HEIGHT - HEADER_HEIGHT - 18)

volatile int8_t audio_buffer[FFT_SIZE];
volatile uint8_t buffer_index = 0;
volatile uint8_t current_channel = 0;

uint8_t bar_height[NUM_BARS] = {0};
uint8_t prev_bar_height[NUM_BARS] = {0};

uint8_t peak_height[NUM_BARS] = {0};
uint8_t prev_peak_height[NUM_BARS] = {0};
uint8_t peak_delay[NUM_BARS] = {0};
uint8_t peak_velocity[NUM_BARS] = {0};

const int8_t W_real[16] = {127, 124, 117, 105, 89, 70, 48, 24, 0, -24, -48, -70, -89, -105, -117, -124};
const int8_t W_imag[16] = {0, -24, -48, -70, -89, -105, -117, -124, -127, -124, -117, -105, -89, -70, -48, -24};

void Hardware_Init(void) {
    DDRB |= (1 << PB7) | (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);
    SPCR = (1 << SPE) | (1 << MSTR);
    SPSR = (1 << SPI2X);

    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    MCUCR |= (1 << ISC01);
    GICR |= (1 << INT0);

    ADMUX = (1 << REFS0) | (1 << ADLAR);
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);

    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 199;
    TIMSK |= (1 << OCIE1A);

    sei();
}

ISR(TIMER1_COMPA_vect) {
    ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect) {
    int16_t sample = ADCH;
    audio_buffer[buffer_index] = (int8_t)(sample - 128);
    buffer_index++;
    if (buffer_index >= FFT_SIZE) {
        buffer_index = 0;
    }
}

ISR(INT0_vect) {
    current_channel ^= 1;
    ADMUX = (1 << REFS0) | (1 << ADLAR) | current_channel;
}

uint16_t int_sqrt(uint32_t n) {
    uint32_t root = 0;
    uint32_t bit = 1UL << 30;
    while (bit > n) bit >>= 2;
    while (bit != 0) {
        if (n >= root + bit) {
            n -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return (uint16_t)root;
}

void calculate_fft(int16_t* fr, int16_t* fi) {
    uint8_t j = 0;
    for (uint8_t i = 0; i < FFT_SIZE - 1; i++) {
        if (i < j) {
            int16_t tr = fr[i]; fr[i] = fr[j]; fr[j] = tr;
            int16_t ti = fi[i]; fi[i] = fi[j]; fi[j] = ti;
        }
        uint8_t k = FFT_SIZE >> 1;
        while (k <= j) { j -= k; k >>= 1; }
        j += k;
    }

    uint8_t step = 1;
    for (uint8_t stage = 0; stage < FFT_STAGES; stage++) {
        uint8_t jump = step << 1;
        uint8_t twiddle_step = (FFT_SIZE >> 1) / step;
        for (uint8_t i = 0; i < step; i++) {
            int8_t wr = W_real[i * twiddle_step];
            int8_t wi = W_imag[i * twiddle_step];
            for (uint8_t k = i; k < FFT_SIZE; k += jump) {
                uint8_t match = k + step;
                int16_t tr = ((int32_t)fr[match] * wr - (int32_t)fi[match] * wi) >> 7;
                int16_t ti = ((int32_t)fi[match] * wr + (int32_t)fr[match] * wi) >> 7;
                fr[match] = fr[k] - tr;
                fi[match] = fi[k] - ti;
                fr[k] += tr;
                fi[k] += ti;
            }
        }
        step = jump;
    }
}

static void draw_spectrum(void)
{
    static uint8_t last_rendered_channel = 0xFF;

    if (current_channel != last_rendered_channel) {
        ILI9341_FillRectangle(2, 2, 80, HEADER_HEIGHT - 1, ILI9341_BLACK);
        if (current_channel == 0) {
            ILI9341_WriteString(2, 2, "MIC: PA0", ILI9341_WHITE, ILI9341_BLACK, 1);
        } else {
            ILI9341_WriteString(2, 2, "JACK: PA1", ILI9341_WHITE, ILI9341_BLACK, 1);
        }
        last_rendered_channel = current_channel;
    }

    for (uint8_t i = 0; i < NUM_BARS; i++)
    {
        // Column i holds FFT bin i (frequency increases with i). This panel's
        // SetWindow() flips the X address (PASET runs right-to-left), which
        // was silently mirroring the whole bar row left<->right even though
        // small fixed-position elements like the header text didn't reveal
        // it. Mirroring the column slot here (not the bin data) restores
        // increasing frequency left-to-right on screen.
        uint8_t col = (NUM_BARS - 1) - i;
        uint16_t x1 = (col * COL_WIDTH) + (COL_GAP / 2);
        uint16_t x2 = x1 + BAR_THICK;

        uint16_t current_top = BAR_Y_BASE - bar_height[i];
        uint16_t prev_top    = BAR_Y_BASE - prev_bar_height[i];

        if (bar_height[i] > prev_bar_height[i]) {
            uint16_t color = (col < 8) ? ILI9341_GREEN : ILI9341_CYAN;
            ILI9341_FillRectangle(x1, current_top, x2, prev_top, color);
        }
        else if (bar_height[i] < prev_bar_height[i]) {
            ILI9341_FillRectangle(x1, prev_top, x2, current_top, ILI9341_BLACK);
        }

        if (prev_peak_height[i] != peak_height[i] && prev_peak_height[i] > 0
                && prev_peak_height[i] > bar_height[i]) {
            uint16_t old_peak_y = BAR_Y_BASE - prev_peak_height[i];
            ILI9341_FillRectangle(x1, old_peak_y - 1, x2, old_peak_y, ILI9341_BLACK);
        }

        if (peak_height[i] > 0) {
            uint16_t new_peak_y = BAR_Y_BASE - peak_height[i];
            ILI9341_FillRectangle(x1, new_peak_y - 1, x2, new_peak_y, ILI9341_RED);
        }

        prev_bar_height[i] = bar_height[i];
        prev_peak_height[i] = peak_height[i];
    }
}

int main(void)
{
    int16_t f_real[FFT_SIZE];
    int16_t f_imag[FFT_SIZE];

    Hardware_Init();
    ILI9341_Init();
    // Orientation fix now lives inside ILI9341.c's SetWindow() (CASET/PASET
    // swap) — no MADCTL hack needed here.
    ILI9341_FillScreen(ILI9341_BLACK);

    while (1)
    {
        cli();
        uint8_t read_idx = buffer_index;
        for (uint8_t i = 0; i < FFT_SIZE; i++) {
            f_real[i] = audio_buffer[read_idx];
            f_imag[i] = 0;
            read_idx++;
            if (read_idx >= FFT_SIZE) {
                read_idx = 0;
            }
        }
        sei();

        calculate_fft(f_real, f_imag);

        for (uint8_t i = 0; i < NUM_BARS; i++) {
            uint32_t mag_sq = (int32_t)f_real[i] * f_real[i] + (int32_t)f_imag[i] * f_imag[i];
            uint16_t mag = int_sqrt(mag_sq);

            uint16_t target_height = (uint16_t)(((uint32_t)mag * 200) / 256);
            if (target_height > BAR_MAX_LEN) {
                target_height = BAR_MAX_LEN;
            }

            if (target_height > bar_height[i]) {
                bar_height[i] = target_height;
            } else {
                bar_height[i] = ((bar_height[i] * 3) + target_height) >> 2;
            }

            if (target_height >= peak_height[i]) {
                peak_height[i] = target_height;
                peak_delay[i] = PEAK_HOLD;
                peak_velocity[i] = 0;
            } else {
                if (peak_delay[i] > 0) {
                    peak_delay[i]--;
                } else if (peak_height[i] > 0) {
                    peak_velocity[i]++;
                    uint8_t drop = peak_velocity[i] >> 1;
                    if (drop == 0) drop = 1;
                    if (peak_height[i] > drop) {
                        peak_height[i] -= drop;
                    } else {
                        peak_height[i] = 0;
                    }
                }

                if (peak_height[i] < bar_height[i]) {
                    peak_height[i] = bar_height[i];
                    peak_velocity[i] = 0;
                }
            }
        }

        draw_spectrum();
        _delay_ms(15);
    }
    return 0;
}