#define F_CPU 16000000UL      // Updated to 16 MHz
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>    // Required for hardware interrupts

#include "SSD1306.h"
#include "Font5x8.h"

/* ------------------------------------------------------------------
   DATA ACQUISITION DEMO — Stage 2
   Goal: Verify deterministic sampling via Timer1 and ADC interrupts.
   This code samples audio inputs at 10 kHz, subtracts the 2.5V DC 
   offset, and prints the raw integer values to the OLED to confirm 
   the hardware and interrupts are working properly before moving 
   to the fixed-point FFT calculations.
   ------------------------------------------------------------------ */

#define FFT_SIZE 32

volatile int8_t audio_buffer[FFT_SIZE]; // Ring buffer for audio samples
volatile uint8_t buffer_index = 0;
volatile uint8_t current_channel = 0;   // 0 for Mic (PA0), 1 for Jack (PA1)

void Hardware_Init(void) {
    // --- 1. Mode Switch Button (INT0 on PD2) ---
    DDRD &= ~(1 << PD2);    // Set PD2 as input
    PORTD |= (1 << PD2);    // Enable internal pull-up resistor
    MCUCR |= (1 << ISC01);  // Trigger on falling edge
    GICR |= (1 << INT0);    // Enable external interrupt 0

    // --- 2. ADC Setup ---
    // REFS0 = AVCC (5V reference)
    // ADLAR = Left Adjust (so we can just read the 8 MSB from ADCH)
    ADMUX = (1 << REFS0) | (1 << ADLAR); // Starts on ADC0 (PA0)
    
    // ADEN = Enable ADC, ADIE = Enable ADC Interrupt
    // ADPS2 & ADPS1 = Prescaler 64 (16MHz / 64 = 250kHz ADC clock)
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);

    // --- 3. Timer1 Setup (10 kHz Nyquist Sampling) ---
    // WGM12 = CTC Mode
    // CS11 = Prescaler 8 (16MHz / 8 = 2MHz timer clock)
    TCCR1B = (1 << WGM12) | (1 << CS11);
    
    // 2MHz / 10kHz = 200 ticks per sample. (200 - 1 = 199)
    OCR1A = 199; 
    
    // OCIE1A = Enable Timer1 Compare Match A Interrupt
    TIMSK |= (1 << OCIE1A);

    // Enable global interrupts
    sei(); 
}

// Timer1 Interrupt: Fires exactly 10,000 times per second
ISR(TIMER1_COMPA_vect) {
    ADCSRA |= (1 << ADSC); // Trigger an ADC conversion
}

// ADC Interrupt: Fires when the conversion is finished
ISR(ADC_vect) {
    // Read only the 8 MSB bits. Subtract 128 to remove the 2.5V DC offset.
    int16_t sample = ADCH;
    audio_buffer[buffer_index] = (int8_t)(sample - 128);

    // Increment and wrap the ring buffer
    buffer_index++;
    if (buffer_index >= FFT_SIZE) {
        buffer_index = 0;
    }
}

// INT0 Interrupt: Fires when you press the button
ISR(INT0_vect) {
    current_channel ^= 1; // Toggle between 0 (PA0) and 1 (PA1)
    
    // Update the multiplexer to change physical input pins
    ADMUX = (1 << REFS0) | (1 << ADLAR) | current_channel;
}

// Function to visually confirm ADC reads on the OLED
static void draw_raw_values(void)
{
    GLCD_Clear();

    // Show which input channel is currently active
    GLCD_GotoXY(2, 0);
    if (current_channel == 0) {
        GLCD_PrintString("MIC (PA0) ACTIVE");
    } else {
        GLCD_PrintString("JACK (PA1) ACTIVE");
    }

    // Safely grab the most recently saved sample
    uint8_t read_idx = (buffer_index == 0) ? (FFT_SIZE - 1) : (buffer_index - 1);
    int8_t latest_sample = audio_buffer[read_idx];

    // Print the raw shifted integer 
    GLCD_GotoXY(2, 20);
    GLCD_PrintString("Raw ADC (-128):");
    
    GLCD_GotoXY(2, 35);
    GLCD_PrintInteger(latest_sample);

    GLCD_Render();
}

int main(void)
{
    GLCD_Setup();
    GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
    
    Hardware_Init(); // Start the 10kHz sampling engine!

    GLCD_Clear();
    GLCD_GotoXY(6, 25);
    GLCD_PrintString("Hardware Init OK");
    GLCD_Render();
    _delay_ms(1200);

    while (1)
    {
        // Render the text instead of the random bars
        draw_raw_values();
        
        // Slower refresh rate (~10 FPS) so the text is readable by human eyes
        _delay_ms(100);   
    }
    return 0;
}