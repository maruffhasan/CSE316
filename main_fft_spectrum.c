/*
 * ATmega32 - 32-point Fixed-Point FFT Audio Spectrum Analyzer on 16x2 LCD
 * ---------------------------------------------------------------------
 * - Mic/analog input on ADC0 (PA0)
 * - 32 real samples captured per frame -> 32-point radix-2 FFT (Q15 fixed
 * point)
 * - Only the 16 positive-frequency bins (N/2) are shown, one per LCD column
 * - Each column height (0-16) is split across the two LCD rows using
 *   8 custom CGRAM "partial bar" characters, giving a 16-level VU/spectrum
 *   look like a real bar-graph LCD spectrum analyzer.
 *
 * Wiring assumptions (same as your LCD_4.c):
 *   LCD D4..D7 -> PD4..PD7
 *   LCD RS     -> PC6
 *   LCD EN     -> PC7
 *   Mic/analog -> PA0 (ADC0), AVCC/AREF wired per datasheet
 *
 * F_CPU = 16MHz assumed.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define D4 eS_PORTD4
#define D5 eS_PORTD5
#define D6 eS_PORTD6
#define D7 eS_PORTD7
#define RS eS_PORTC6
#define EN eS_PORTC7

#include "lcd.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>

/* ------------------------------------------------------------------ */
/*  >>> TUNABLE: highest frequency shown on the spectrum <<<           */
/* ------------------------------------------------------------------ */
/*
 * MAX_FREQ_KHZ sets the top of the displayed spectrum, in kHz.
 *   MAX_FREQ_KHZ = 2   -> spectrum covers 0..2kHz
 *   MAX_FREQ_KHZ = 10  -> spectrum covers 0..10kHz
 *
 * By the Nyquist criterion the ADC has to sample at 2x this rate, so
 * the code below derives the actual sampling period from it automatically
 * - you only ever need to change this one line.
 *
 * Practical ceiling: a single ADC conversion at the prescaler used here
 * (ADPS = /128, ADC clock = 125kHz) takes ~104us on its own, which by
 * itself limits sampling to roughly 9.6kHz (~4.8kHz max displayable
 * frequency). Asking for a higher MAX_FREQ_KHZ than that will just make
 * SAMPLE_DELAY_US bottom out at 0 and you'll be capped by the ADC's own
 * conversion speed rather than actually sampling faster. If you need to
 * go higher, lower the ADC prescaler (ADPS bits in ADC_Init) as well.
 */
#define MAX_FREQ_KHZ 20

/* Derived sampling period (microseconds) = 1 / (2 * MAX_FREQ_KHZ * 1000) */
#define SAMPLE_PERIOD_US (500UL / MAX_FREQ_KHZ)

/* ------------------------------------------------------------------ */
/*  ADC speed - this is what actually limits how high MAX_FREQ_KHZ      */
/*  can go. If the spectrum won't go above ~5kHz and instead aliases    */
/*  ("wraps around"), this is the value to change, not MAX_FREQ_KHZ.    */
/* ------------------------------------------------------------------ */
/*
 * Every ADC_Read() takes ~13 ADC clock cycles no matter what. At the
 * original /128 prescaler (125kHz ADC clock) that's ~104us PER SAMPLE,
 * all by itself - which caps real sampling at ~9.6kHz (Nyquist ~4.8kHz)
 * regardless of how small SAMPLE_DELAY_US gets. That fixed cost was
 * your ceiling.
 *
 * Lower ADC_PRESCALER_DIV to shrink that per-sample cost. The datasheet
 * recommends keeping the ADC clock <=200kHz for full 10-bit accuracy;
 * going faster trades a little resolution/noise for speed, which is a
 * totally fine tradeoff for a bar-graph spectrum display.
 *
 *   ADC_PRESCALER_DIV   ADC clock   ~conversion time   in datasheet spec?
 *         128              125kHz         104us              yes
 *          64              250kHz          52us              no (2x over)
 *          32              500kHz          26us              no (2.5x over)
 *          16               1MHz           13us              no (5x over, still
 * works)
 *
 * Start with 32. Only drop to 16 if you need MAX_FREQ_KHZ beyond ~19
 * and can tolerate a noisier reading.
 */
#define ADC_PRESCALER_DIV 32UL

#if ADC_PRESCALER_DIV == 128
#define ADPS_BITS ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))
#elif ADC_PRESCALER_DIV == 64
#define ADPS_BITS ((1 << ADPS2) | (1 << ADPS1))
#elif ADC_PRESCALER_DIV == 32
#define ADPS_BITS ((1 << ADPS2) | (1 << ADPS0))
#elif ADC_PRESCALER_DIV == 16
#define ADPS_BITS ((1 << ADPS2))
#elif ADC_PRESCALER_DIV == 8
#define ADPS_BITS ((1 << ADPS1) | (1 << ADPS0))
#else
#error "ADC_PRESCALER_DIV must be one of 128, 64, 32, 16, 8"
#endif

#define ADC_CLOCK_HZ (F_CPU / ADC_PRESCALER_DIV)

/* Approx. time a single ADC_Read() burns just doing the conversion
   itself (~13 ADC clocks), recomputed automatically from whichever
   prescaler you picked above. Subtract this from the target period so
   the *extra* delay below brings the total period up to
   SAMPLE_PERIOD_US, rather than adding on top of it. */
#define ADC_CONVERSION_US (13000000UL / ADC_CLOCK_HZ)

#if SAMPLE_PERIOD_US > ADC_CONVERSION_US
#define SAMPLE_DELAY_US (SAMPLE_PERIOD_US - ADC_CONVERSION_US)
#else
#define SAMPLE_DELAY_US 0UL
#endif

/* ------------------------------------------------------------------ */
/*  Config                                                             */
/* ------------------------------------------------------------------ */
#define N 32        /* FFT size                                */
#define NUM_BARS 16 /* N/2 -> one LCD column per bin            */
#define ADC_CH 0    /* PA0 = mic input                          */

/* ------------------------------------------------------------------ */
/*  ADC                                                                 */
/* ------------------------------------------------------------------ */
void ADC_Init(void) {
  DDRA &= ~(1 << ADC_CH); /* PA0 as input      */
  ADMUX = (1 << REFS0);   /* AVcc ref, ch 0    */
  ADCSRA = (1 << ADEN) | ADPS_BITS;
  /* Prescaler set by ADC_PRESCALER_DIV above (currently gives an
     ADC clock of ADC_CLOCK_HZ, and ~ADC_CONVERSION_US per sample). */
}

uint16_t ADC_Read(uint8_t ch) {
  ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  return ADC; /* 0..1023          */
}

/* ------------------------------------------------------------------ */
/*  Fixed point (Q15) 32-point FFT                                     */
/* ------------------------------------------------------------------ */
static int16_t re[N];
static int16_t im[N];

/* Twiddle factors W_N^k = cos(2*pi*k/N) - j*sin(2*pi*k/N), k = 0..N/2-1 */
static const int16_t twR[16] PROGMEM = {
    32767, 32137, 30274,  27245,  23170,  18204,  12539,  6393,
    0,     -6393, -12539, -18204, -23170, -27245, -30274, -32137};
static const int16_t twI[16] PROGMEM = {
    0,     6393,  12539, 18204, 23170, 27245, 30274, 32137,
    32767, 32137, 30274, 27245, 23170, 18204, 12539, 6393};

static void fft32(void) {
  uint8_t i, j, k, len, half, step;
  int16_t tmp;

  /* --- bit-reversal permutation --- */
  for (i = 1, j = 0; i < N; i++) {
    uint8_t bit = N >> 1;
    for (; j & bit; bit >>= 1)
      j ^= bit;
    j ^= bit;
    if (i < j) {
      tmp = re[i];
      re[i] = re[j];
      re[j] = tmp;
      tmp = im[i];
      im[i] = im[j];
      im[j] = tmp;
    }
  }

  /* --- iterative Cooley-Tukey, with per-stage >>1 scaling to avoid overflow
   * --- */
  for (len = 2; len <= N; len <<= 1) {
    half = len >> 1;
    step = N / len;
    for (i = 0; i < N; i += len) {
      for (k = 0, j = i; k < half; k++, j++) {
        uint8_t idx = k * step;
        int16_t wr = (int16_t)pgm_read_word(&twR[idx]);
        int16_t wi = (int16_t)pgm_read_word(&twI[idx]);
        int16_t reo = re[j + half];
        int16_t imo = im[j + half];

        int32_t trr = ((int32_t)wr * reo + (int32_t)wi * imo) >> 15;
        int32_t tii = ((int32_t)wr * imo - (int32_t)wi * reo) >> 15;

        re[j + half] = (int16_t)((re[j] - trr) >> 1);
        im[j + half] = (int16_t)((im[j] - tii) >> 1);
        re[j] = (int16_t)((re[j] + trr) >> 1);
        im[j] = (int16_t)((im[j] + tii) >> 1);
      }
    }
  }
}

/* ------------------------------------------------------------------ */
/*  Magnitude -> 0..16 bar levels                                      */
/* ------------------------------------------------------------------ */
static uint8_t bar[NUM_BARS];

static void compute_bars(void) {
  uint16_t mag[NUM_BARS];
  uint16_t maxMag = 1;
  uint8_t i;

  for (i = 0; i < NUM_BARS; i++) {
    int16_t a = re[i];
    if (a < 0)
      a = -a;
    int16_t b = im[i];
    if (b < 0)
      b = -b;
    /* cheap magnitude approximation: max + 0.5*min */
    uint16_t m = (a > b) ? (uint16_t)(a + (b >> 1)) : (uint16_t)(b + (a >> 1));
    mag[i] = m;
    if (m > maxMag)
      maxMag = m;
  }

  for (i = 0; i < NUM_BARS; i++) {
    uint32_t lvl =
        (uint32_t)mag[i] * 16 / maxMag; /* auto-scale to tallest bin */
    if (lvl > 16)
      lvl = 16;
    bar[i] = (uint8_t)lvl;
  }
}

/* ------------------------------------------------------------------ */
/*  LCD custom "partial bar" characters (CGRAM 0..7)                   */
/* ------------------------------------------------------------------ */
static const uint8_t barChars[8][8] PROGMEM = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F}, /* 1/8 */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F}, /* 2/8 */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F}, /* 3/8 */
    {0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F}, /* 4/8 */
    {0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, /* 5/8 */
    {0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, /* 6/8 */
    {0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, /* 7/8 */
    {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, /* 8/8 full block */
};

/* Send a full 8-bit command (RS=0) as two nibbles - needed for the
   0x40+ CGRAM-address command, which Lcd4_Cmd() alone can't build. */
static void Lcd4_Cmd8(uint8_t a) {
  pinChange(RS, 0);
  Lcd4_Port(a >> 4);
  pinChange(EN, 1);
  _delay_us(50);
  pinChange(EN, 0);
  _delay_us(50);
  Lcd4_Port(a & 0x0F);
  pinChange(EN, 1);
  _delay_us(50);
  pinChange(EN, 0);
  _delay_us(50);
}

static void Lcd_Load_Bar_Chars(void) {
  uint8_t i, j;
  for (i = 0; i < 8; i++) {
    Lcd4_Cmd8(0x40 | (i << 3)); /* set CGRAM address */
    for (j = 0; j < 8; j++)
      Lcd4_Write_Char((char)pgm_read_byte(&barChars[i][j]));
  }
  Lcd4_Cmd8(0x80); /* back to DDRAM addr 0 */
}

/* ------------------------------------------------------------------ */
/*  Draw the 16 bars across both LCD rows                              */
/* ------------------------------------------------------------------ */
static void draw_bars(void) {
  uint8_t i, lvl, top, bot;

  for (i = 0; i < NUM_BARS; i++) {
    lvl = bar[i];
    bot = (lvl > 8) ? 8 : lvl;
    top = (lvl > 8) ? (uint8_t)(lvl - 8) : 0;

    Lcd4_Set_Cursor(1, i + 1);
    Lcd4_Write_Char(top == 0 ? ' ' : (char)(top - 1));

    Lcd4_Set_Cursor(2, i + 1);
    Lcd4_Write_Char(bot == 0 ? ' ' : (char)(bot - 1));
  }
}

/* ------------------------------------------------------------------ */
/*  Main                                                                */
/* ------------------------------------------------------------------ */
int main(void) {
  uint8_t i;

  DDRC = 0xFF; /* LCD RS/EN */
  DDRD = 0xFF; /* LCD D4-D7 */
  /* PA0 left as input for ADC (see ADC_Init) */

  Lcd4_Init();
  ADC_Init();
  Lcd_Load_Bar_Chars();
  Lcd4_Clear();

  while (1) {
    /* ---- capture 32 samples, centered around 0 ---- */
    for (i = 0; i < N; i++) {
      uint16_t s = ADC_Read(ADC_CH); /* 0..1023 */
      re[i] = (int16_t)s - 512;      /* remove DC bias      */
      im[i] = 0;
      _delay_us(SAMPLE_DELAY_US); /* set via MAX_FREQ_KHZ */
                                  /* at the top of the   */
                                  /* file                */
    }

    fft32();
    compute_bars();
    draw_bars();
  }
}
