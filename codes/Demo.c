#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "ILI9341.h"
// ILI9341 Color Definitions (RGB565 format)
#define ILI9341_BLACK   0x0000
#define ILI9341_WHITE   0xFFFF
#define ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_RED     0xF800

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
#define SCREEN_W     320  
#define SCREEN_H     240
#define BAR_WIDTH    12
#define BAR_GAP      3
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
// --- Hardware Initialization ---
void Hardware_Init(void) {
    // 1. SPI Protocol Setup 
    // PB7 (SCK), PB5 (MOSI), PB4 (CS), PB3 (D/C), PB2 (RST) ->output 
    DDRB |= (1 << PB7) | (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);
    
    // spi enable,master mode
    SPCR = (1 << SPE) | (1 << MSTR); 
    SPSR = (1 << SPI2X); 

    // 2. previous interrupt and adc
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
    // Clear the display with Black color
    ILI9341_FillScreen(ILI9341_BLACK); 

    // Print the active channel (Assuming ILI9341_WriteString takes: X, Y, Text, TextColor, BgColor, Size)
    if (current_channel == 0) {
        ILI9341_WriteString(2, 0, "MIC: PA0", ILI9341_WHITE, ILI9341_BLACK, 1);
    } else {
        ILI9341_WriteString(2, 0, "JACK: PA1", ILI9341_WHITE, ILI9341_BLACK, 1);
    }

    // Loop through all 16 frequency bins[cite: 1]
    for (uint8_t i = 0; i < NUM_BARS; i++)
    {
        uint16_t x1 = i * (BAR_WIDTH + BAR_GAP);
        uint16_t x2 = x1 + BAR_WIDTH - 1;
        uint16_t y2 = BASELINE;
        
        // 1. Draw the moving bar
        if (bar_height[i] > 0) {
            uint16_t y1 = BASELINE - bar_height[i];
            
            if (i < 8) {
                // Low frequencies: Solid Green bars
                // Assuming function format: (x1, y1, x2, y2, color)
                ILI9341_FillRectangle(x1, y1, x2, y2, ILI9341_GREEN);
            } else {
                // High frequencies: Hollow Cyan bars
                ILI9341_DrawRectangle(x1, y1, x2, y2, ILI9341_CYAN);
            }
        }

        // 2. Draw the floating peak dot (Kinetic Gravity)
        if (peak_height[i] > 0) {
            uint16_t peak_y = BASELINE - peak_height[i];
            
            // Draw a thick Red line for the falling dot
            ILI9341_DrawLine(x1, peak_y, x2, peak_y, ILI9341_RED);
            if (peak_y > 0) {
                ILI9341_DrawLine(x1, peak_y - 1, x2, peak_y - 1, ILI9341_RED);
            }
        }
    }
}
int main(void)
{
    int16_t f_real[FFT_SIZE];
    int16_t f_imag[FFT_SIZE];
    Hardware_Init();
    ILI9341_Init();
    ILI9341_FillScreen(ILI9341_BLACK); 
    
    

    while (1)
    {
        // 1. Snapshot the audio buffer safely
        cli(); 
        uint8_t read_idx = buffer_index; 
        for(uint8_t i = 0; i < FFT_SIZE; i++) {
            f_real[i] = audio_buffer[read_idx];
            f_imag[i] = 0; 
            
            read_idx++;
            if (read_idx >= FFT_SIZE) {
                read_idx = 0; 
            }
        }
        sei();

        // 2. Perform the FFT[cite: 1]
        calculate_fft(f_real, f_imag);

        // 3. Calculate magnitudes and process ADVANCED PHYSICS[cite: 1]
        for(uint8_t i = 0; i < NUM_BARS; i++) {
            uint32_t mag_squared = (int32_t)f_real[i]*f_real[i] + (int32_t)f_imag[i]*f_imag[i];
            uint16_t mag = int_sqrt(mag_squared);
            
            // --- scaling update ---
            uint16_t target_height = mag >> 1;             
            if (target_height > MAX_BAR_H) {
                target_height = MAX_BAR_H;
            }
            
            // --- FLUID BARS (Exponential Moving Average) ---[cite: 1]
            if (target_height > bar_height[i]) {
                bar_height[i] = target_height; 
            } else {
                bar_height[i] = ((bar_height[i] * 3) + target_height) >> 2; 
            }

            // --- KINETIC GRAVITY (Falling Dots) ---[cite: 1]
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
            }
        }

        // 4. Render to Color TFT[cite: 1]
        draw_spectrum();
        
        // Slightly faster refresh rate (~33 FPS) for smoother animation[cite: 1]
        _delay_ms(30);   
    }
    
    return 0;
}