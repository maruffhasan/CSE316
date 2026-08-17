

// #define F_CPU 10000000UL
#include <avr/io.h>
#include <util/delay.h>

#include "SSD1306.h"
#include "Font5x8.h"

/* ------------------------------------------------------------------
   PRELIMINARY DEMO — Stage 1
   Goal: just prove we can render a 16-bar spectrum-style bargraph on
   the SSD1306 in SimulIDE. No ADC, no FFT yet — bar heights are
   generated with a tiny pseudo-random walk so the display animates
   the way real audio data eventually will.
   ------------------------------------------------------------------ */

#define NUM_BARS     16      // will map to 16 FFT bins later
#define BAR_WIDTH    6
#define BAR_GAP      2
#define SCREEN_H     64       // change to 32 if you're using a 128x32 module
#define BASELINE     (SCREEN_H - 1)
#define MAX_BAR_H    (SCREEN_H - 12)   // leave room for the title text

static uint8_t bar_height[NUM_BARS] = {5,8,12,20,10,15,6,4,18,25,30,10,8,35,40,12};
static uint16_t rng_state = 12345;

/* --- stand-in "signal" generator -----------------------------------
   Replace this later with real magnitude values coming from the FFT.
   For now it's a simple 16-bit LCG that nudges each bar toward a new
   random target every frame, clamped so the motion looks smooth
   instead of jumping around like static. */
static uint8_t pseudo_random(uint8_t max)
{
    rng_state = (uint16_t)(rng_state * 25173U + 13849U);
    return (uint8_t)((rng_state >> 8) % (max + 1));
}

static void update_bars(void)
{
    for (uint8_t i = 0; i < NUM_BARS; i++)
    {
        uint8_t target = pseudo_random(MAX_BAR_H);

        if (target > bar_height[i])
        {
            uint8_t step = target - bar_height[i];
            bar_height[i] += (step > 6) ? 6 : step;   // rise fast
        }
        else
        {
            uint8_t step = bar_height[i] - target;
            bar_height[i] -= (step > 8) ? 8 : step;   // fall a bit faster
        }

        if (bar_height[i] > MAX_BAR_H)
            bar_height[i] = MAX_BAR_H;
    }
}

static void draw_spectrum(void)
{
    GLCD_Clear();

    GLCD_GotoXY(2, 0);
    GLCD_PrintString("SPECTRUM DEMO");

    for (uint8_t i = 0; i < NUM_BARS; i++)
    {
        uint8_t x1 = i * (BAR_WIDTH + BAR_GAP);
        uint8_t x2 = x1 + BAR_WIDTH - 1;
        uint8_t y2 = BASELINE;
        uint8_t y1 = y2 - bar_height[i];

        GLCD_FillRectangle(x1, y1, x2, y2, GLCD_Black);
        GLCD_DrawRectangle(x1, y1, x2, y2, GLCD_Black);
    }

    GLCD_Render();
}

int main(void)
{
    GLCD_Setup();
    GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);

    GLCD_Clear();
    GLCD_GotoXY(6, 25);
    GLCD_PrintString("Spectrum Visualizer");
    GLCD_Render();
    _delay_ms(1200);

    while (1)
    {
        update_bars();
        draw_spectrum();
        _delay_ms(80);   // ~12 FPS refresh, plenty for a first look
    }

    return 0;
}