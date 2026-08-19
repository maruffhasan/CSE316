#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "SSD1306.h"
#include "Font5x8.h"

/* ------------------------------------------------------------------
   FULL AUDIO SPECTRUM ANALYZER — Stage 3 (Final)
   Goal: 10kHz deterministic sampling via Timer1 + ADC, followed by 
   a 32-point fixed-point Radix-2 FFT to extract frequency bins.
   The magnitudes are calculated and pushed to the OLED bargraph,
   featuring peak-hold decay and falling dots for visual dynamics.
   ------------------------------------------------------------------ */

#define FFT_SIZE     32
#define FFT_STAGES   5
#define NUM_BARS     16
#define BAR_WIDTH    6
#define BAR_GAP      2
#define SCREEN_H     64
#define BASELINE     (SCREEN_H - 1)
#define MAX_BAR_H    (SCREEN_H - 12)
#define PEAK_HOLD    3    // Number of frames the dot hovers before falling

volatile int8_t audio_buffer[FFT_SIZE];
volatile uint8_t buffer_index = 0;
volatile uint8_t current_channel = 0;

uint8_t bar_height[NUM_BARS] = {0};
uint8_t peak_height[NUM_BARS] = {0}; // Tracks the falling dots
uint8_t peak_delay[NUM_BARS] = {0};  // Timer for the hover effect 
uint8_t peak_velocity[NUM_BARS] = {0}; // NEW: Tracks downward momentum for gravity

// Twiddle factors (Sine/Cosine table) for N=32, scaled by 128
const int8_t W_real[16] = {127, 124, 117, 105, 89, 70, 48, 24, 0, -24, -48, -70, -89, -105, -117, -124};
const int8_t W_imag[16] = {0, -24, -48, -70, -89, -105, -117, -124, -127, -124, -117, -105, -89, -70, -48, -24};

// --- Hardware Initialization ---
void Hardware_Init(void) {
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

// --- Digital Signal Processing ---

// Fast integer square root for magnitude calculation
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

// 32-Point In-Place Radix-2 FFT
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

// --- Display Rendering ---
/*
static void draw_spectrum(void)
{
    GLCD_Clear();

    GLCD_GotoXY(2, 0);
    if (current_channel == 0) {
        GLCD_PrintString("MIC: PA0");
    } else {
        GLCD_PrintString("JACK: PA1");
    }

    for (uint8_t i = 0; i < NUM_BARS; i++)
    {
        uint8_t x1 = i * (BAR_WIDTH + BAR_GAP);
        uint8_t x2 = x1 + BAR_WIDTH - 1;
        uint8_t y2 = BASELINE;
        
        // 1. Draw the solid moving bar
        if (bar_height[i] > 0) {
            uint8_t y1 = BASELINE - bar_height[i];
            GLCD_FillRectangle(x1, y1, x2, y2, GLCD_Black);
        }

        // 2. Draw the floating peak dot
        if (peak_height[i] > 0) {
            uint8_t peak_y = BASELINE - peak_height[i];
            // Draw a 2-pixel thick line for the falling dot so it is clearly visible
            GLCD_DrawLine(x1, peak_y, x2, peak_y, GLCD_Black);
            if (peak_y > 0) {
                GLCD_DrawLine(x1, peak_y - 1, x2, peak_y - 1, GLCD_Black);
            }
        }
    }

    GLCD_Render();
}
*/
static void draw_spectrum(void)
{
    // Clear the display buffer before drawing the new frame
    GLCD_Clear();

    // Print the currently active input channel at the top-left corner
    GLCD_GotoXY(2, 0);
    if (current_channel == 0) {
        GLCD_PrintString("MIC: PA0");
    } else {
        GLCD_PrintString("JACK: PA1");
    }

    // Loop through all the frequency bins (16 bars) to draw them
    for (uint8_t i = 0; i < NUM_BARS; i++)
    {
        // Calculate the X coordinates for the current bar
        uint8_t x1 = i * (BAR_WIDTH + BAR_GAP);
        uint8_t x2 = x1 + BAR_WIDTH - 1;
        
        // The bottom of the bar is anchored to the BASELINE of the screen
        uint8_t y2 = BASELINE;
        
        // 1. Draw the moving bar representing the current frequency magnitude
        if (bar_height[i] > 0) {
            // Calculate the top Y coordinate based on the bar height
            uint8_t y1 = BASELINE - bar_height[i];
            
            // Visual separation: Low frequencies (first 8 bars) are solid
            if (i < 8) {
                GLCD_FillRectangle(x1, y1, x2, y2, GLCD_Black);
            } 
            // High frequencies (remaining 8 bars) are hollow (outline only)
            else {
                GLCD_DrawRectangle(x1, y1, x2, y2, GLCD_Black);
            }
        }

        // 2. Draw the floating peak dot (Kinetic Gravity effect)
        if (peak_height[i] > 0) {
            // Calculate the Y coordinate for the peak dot
            uint8_t peak_y = BASELINE - peak_height[i];
            
            // Draw a 2-pixel thick horizontal line so the falling dot is clearly visible
            GLCD_DrawLine(x1, peak_y, x2, peak_y, GLCD_Black);
            if (peak_y > 0) {
                GLCD_DrawLine(x1, peak_y - 1, x2, peak_y - 1, GLCD_Black);
            }
        }
    }

    // Push the updated buffer to the OLED screen to make it visible
    GLCD_Render();
}
int main(void)
{
    int16_t f_real[FFT_SIZE];
    int16_t f_imag[FFT_SIZE];

    GLCD_Setup();
    GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
    Hardware_Init(); 

    while (1)
    {
        // 1. Snapshot the audio buffer safely (Unwrapping the ring buffer)
        cli(); 
        uint8_t read_idx = buffer_index; // The current index holds the oldest sample
        for(uint8_t i = 0; i < FFT_SIZE; i++) {
            f_real[i] = audio_buffer[read_idx];
            f_imag[i] = 0; 
            
            read_idx++;
            if (read_idx >= FFT_SIZE) {
                read_idx = 0; // Wrap around
            }
        }
        sei();

        // 2. Perform the FFT
        calculate_fft(f_real, f_imag);

        // 3. Calculate magnitudes and process ADVANCED PHYSICS
        for(uint8_t i = 0; i < NUM_BARS; i++) {
            uint32_t mag_squared = (int32_t)f_real[i]*f_real[i] + (int32_t)f_imag[i]*f_imag[i];
            uint16_t mag = int_sqrt(mag_squared);
            
            uint8_t target_height = mag >> 3; 
            if (target_height > MAX_BAR_H) {
                target_height = MAX_BAR_H;
            }
            
            // --- FLUID BARS (Exponential Moving Average) ---
            if (target_height > bar_height[i]) {
                // Punch up instantly for fast attacks (drums/bass)
                bar_height[i] = target_height; 
            } else {
                // Glide down smoothly (Current = 75% Current + 25% Target)
                // This prevents the jittery "teleporting" look
                bar_height[i] = ((bar_height[i] * 3) + target_height) >> 2; 
            }

            // --- KINETIC GRAVITY (Falling Dots) ---
            if (target_height >= peak_height[i]) {
                // Push dot up and reset physics
                peak_height[i] = target_height;
                peak_delay[i] = PEAK_HOLD; 
                peak_velocity[i] = 0; // Reset momentum
            } else {
                if (peak_delay[i] > 0) {
                    peak_delay[i]--; // Hover in place at the top
                } else if (peak_height[i] > 0) {
                    // Gravity accelerates over time
                    peak_velocity[i]++; 
                    
                    // Bit-shift divides velocity by 2 to control the fall speed
                    uint8_t drop = peak_velocity[i] >> 1; 
                    if (drop == 0) drop = 1; // Minimum drop of 1 pixel
                    
                    if (peak_height[i] > drop) {
                        peak_height[i] -= drop;
                    } else {
                        peak_height[i] = 0;
                    }
                }
            }
        }

        // 4. Render to OLED
        draw_spectrum();
        
        // Slightly faster refresh rate (~33 FPS) for smoother animation
        _delay_ms(30);   
    }
    return 0;
}