// ATmega8 @ 1 MHz : 32-sample capture -> 32-point FFT -> 16 positive bins -> UART
//
// Packets sent to the Arduino:
//   0xFE + 16 bytes  : FFT magnitude of bins 1..16   (bin k = k * MAX_FREQ_HZ / 16)
//   0xFF + 32 bytes  : raw waveform (when PB0 is LOW)
//
// MAX_FREQ_HZ sets the REAL sampling rate (fs = 2 * MAX_FREQ_HZ), so the 16
// bins always span exactly 0 .. MAX_FREQ_HZ.  Use the SAME value in arduino.ino.
//
// Inputs:  PB1 LOW  -> ADC0 (mic)      PB1 HIGH -> ADC1 (jack)
// Mode:    PB0 LOW  -> waveform mode   PB0 HIGH -> FFT mode

#define F_CPU 1000000UL
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>

#define MAX_FREQ_HZ 2000UL

#define N 32
#define UBRR_VALUE 1 // U2X, 1 MHz -> 62500 baud

#define SAMPLE_PERIOD_US (1000000UL / (2UL * MAX_FREQ_HZ))

#if (SAMPLE_PERIOD_US < 40UL)
#error "MAX_FREQ_HZ too high for a 1 MHz ATmega (limit is about 12500 Hz)"
#elif (SAMPLE_PERIOD_US >= 260UL)
#define ADC_PS (1 << ADPS2)
#elif (SAMPLE_PERIOD_US >= 130UL)
#define ADC_PS ((1 << ADPS1) | (1 << ADPS0))
#elif (SAMPLE_PERIOD_US >= 70UL)
#define ADC_PS (1 << ADPS1)
#else
#define ADC_PS (1 << ADPS0)
#endif

// ---- tuning ----------------------------------------------------------------
#define GAIN_MIC 3      // software gain, mic  (try 1..4)
#define GAIN_JACK 7     // software gain, jack (try 4..10)
// Wave-mode gain (Arduino WAVE_GAIN must be 1, the ATmega does the scaling now)
#define GAIN_WAVE_MIC 1   // same as the old Arduino WAVE_GAIN
#define GAIN_WAVE_JACK 1 // jack signal is weaker -> more gain
#define BIN1_EXTRA 2    // extra noise margin for the lowest bin (bar 0)
#define DEADBAND 1      // output values <= this are shown as 0 (mic only)
#define CAL_FRAMES 16   // frames averaged for the mic noise profile
#define NOISE_MARGIN 1  // headroom above the measured noise PEAK (raise if bar 0 still flickers)

static uint8_t noise_floor[16]; // per-bin noise (index 0 = bin 1)

const int8_t Sinewave[64] = {
    0, 12, 25, 37, 49, 60, 71, 81, 90, 98, 106,
    112, 117, 122, 125, 126, 127, 126, 125, 122, 117, 112,
    106, 98, 90, 81, 71, 60, 49, 37, 25, 12, 0,
    -12, -25, -37, -49, -60, -71, -81, -90, -98, -106, -112,
    -117, -122, -125, -126, -127, -126, -125, -122, -117, -112, -106,
    -98, -90, -81, -71, -60, -49, -37, -25, -12};

static void UART_Init(void)
{
  UCSRA = (1 << U2X);
  UBRRH = 0;
  UBRRL = UBRR_VALUE;
  UCSRB = (1 << TXEN);
  UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

static void UART_Send(uint8_t c)
{
  while (!(UCSRA & (1 << UDRE)))
    ;
  UDR = c;
}

static void ADC_Init(void)
{
  ADMUX = (1 << REFS0) | (1 << ADLAR);
  ADCSRA = (1 << ADEN) | ADC_PS;
}

// Timer1 free-runs in CTC mode at 1 MHz -> one compare match every sample period
static void Timer_Init(void)
{
  OCR1A = (uint16_t)(SAMPLE_PERIOD_US - 1);
  TCNT1 = 0;
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS10);
  TIFR = (1 << OCF1A);
}

static void capture(int16_t *out, uint8_t channel)
{
  ADMUX = (1 << REFS0) | (1 << ADLAR) | (channel & 0x07);

  // dummy conversions to let the mux / S&H settle after a channel change
  for (uint8_t d = 0; d < 3; d++)
  {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
      ;
  }

  TIFR = (1 << OCF1A);
  for (uint8_t i = 0; i < N; i++)
  {
    while (!(TIFR & (1 << OCF1A))) // wait for the exact sample instant
      ;
    TIFR = (1 << OCF1A);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
      ;
    out[i] = (int8_t)(ADCH - 128);
  }
}

// In-place radix-2 FFT, N = 32.  Twiddle index into the 64-entry table is m*32/l.
static void compute_fft(int16_t fr[], int16_t fi[])
{
  uint8_t i, j, k, m, l, step, sh;
  int16_t t, tr, ti;

  j = 0;
  for (i = 0; i < N - 1; i++)
  { // bit reversal
    if (i < j)
    {
      t = fr[j]; fr[j] = fr[i]; fr[i] = t;
      t = fi[j]; fi[j] = fi[i]; fi[i] = t;
    }
    k = N / 2;
    while (k <= j)
    {
      j -= k;
      k >>= 1;
    }
    j += k;
  }

  sh = 5; // l=1 -> 5, l=2 -> 4 ... l=16 -> 1   (so m << sh == m*32/l)
  for (l = 1; l < N; l = step)
  {
    step = l << 1;
    for (m = 0; m < l; m++)
    {
      uint8_t w = (uint8_t)(m << sh);
      int16_t wr = Sinewave[(w + 16) & 63];
      int16_t wi = -Sinewave[w & 63];
      for (i = m; i < N; i += step)
      {
        j = i + l;
        tr = (int16_t)((((int32_t)wr * fr[j]) - ((int32_t)wi * fi[j])) >> 7);
        ti = (int16_t)((((int32_t)wr * fi[j]) + ((int32_t)wi * fr[j])) >> 7);
        fr[j] = fr[i] - tr;
        fi[j] = fi[i] - ti;
        fr[i] += tr;
        fi[i] += ti;
      }
    }
    sh--;
  }
}

static int16_t clamp127(int32_t v)
{
  if (v > 127)
    return 127;
  if (v < -127)
    return -127;
  return (int16_t)v;
}

// Capture one frame, remove DC, apply gain, Hann window, FFT -> mag[0..15] (bins 1..16)
static void get_spectrum(int16_t *real, int16_t *imag, uint8_t ch,
                         uint8_t gain, uint16_t *mag)
{
  capture(real, ch);

  int16_t sum = 0;
  for (uint8_t i = 0; i < N; i++)
  {
    sum += real[i];
    imag[i] = 0;
  }
  // mean*gain with full precision (rounded): no fractional-offset leftover,
  // which the Hann window would otherwise leak into bin 1 (bar 0)
  int32_t mean_g = ((int32_t)sum * gain + 16) >> 5;

  for (uint8_t i = 0; i < N; i++)
  {
    int16_t w = (127 - Sinewave[(2 * i + 16) & 63]) >> 1;
    int16_t x = clamp127((int32_t)real[i] * gain - mean_g);
    real[i] = (int16_t)(((int32_t)x * w) >> 7);
  }
  compute_fft(real, imag);

  for (uint8_t i = 1; i <= 16; i++)
    mag[i - 1] = ((uint16_t)abs(real[i]) + (uint16_t)abs(imag[i])) >> 3;
}

// Measure the idle mic spectrum (keep the room quiet for ~1 s).
// Uses the PEAK seen per bin (not the average) so random noise spikes stay hidden.
static void calibrate_noise(int16_t *real, int16_t *imag)
{
  uint16_t peak[16] = {0};
  uint16_t mag[16];

  for (uint8_t f = 0; f < CAL_FRAMES; f++)
  {
    get_spectrum(real, imag, 0, GAIN_MIC, mag);
    for (uint8_t b = 0; b < 16; b++)
      if (mag[b] > peak[b])
        peak[b] = mag[b];
  }
  for (uint8_t b = 0; b < 16; b++)
  {
    uint16_t n = peak[b] + NOISE_MARGIN + (b == 0 ? BIN1_EXTRA : 0);
    noise_floor[b] = (n > 252) ? 252 : (uint8_t)n;
  }
}

int main(void)
{
  int16_t real[N], imag[N];
  uint16_t mag[16];
  uint8_t last_pb0 = 1;
  uint8_t active_adc_channel;
  uint8_t calibrated_for = 0xFF; // forces calibration on first FFT frame

  // PB0 and PB1 as inputs; pull-up only on PB0 (PB1 has no pull-up)
  DDRB &= ~((1 << PB0) | (1 << PB1));
  PORTB |= (1 << PB0);
  PORTB &= ~(1 << PB1);

  UART_Init();
  ADC_Init();
  Timer_Init();

  while (1)
  {
    active_adc_channel = (PINB & (1 << PB1)) ? 1 : 0; // 0 = mic, 1 = jack
    uint8_t gain = (active_adc_channel == 0) ? GAIN_MIC : GAIN_JACK;

    uint8_t current_pb0 = (PINB & (1 << PB0)) ? 1 : 0;

    // PB0 just grounded: Arduino likely dipped/reset, stay silent for 2 s
    if (last_pb0 == 1 && current_pb0 == 0)
    {
      for (uint8_t d = 0; d < 20; d++)
        _delay_ms(100);
      last_pb0 = current_pb0;
      continue; // discard the stale frame, start fresh
    }
    last_pb0 = current_pb0;

    if (current_pb0 == 0)
    {
      // ---------- WAVE MODE ----------
      capture(real, active_adc_channel);

      int16_t sum = 0;
      for (uint8_t i = 0; i < N; i++)
        sum += real[i];
      int16_t mean = sum >> 5;

      uint8_t wgain = (active_adc_channel == 0) ? GAIN_WAVE_MIC : GAIN_WAVE_JACK;

      UART_Send(0xFF);
      for (uint8_t i = 0; i < N; i++)
      {
        int16_t v = (int16_t)(((int32_t)(real[i] - mean) * wgain)) + 128;
        if (v < 0)
          v = 0;
        if (v > 252)
          v = 252;
        UART_Send((uint8_t)v);
      }
      _delay_ms(80);

      calibrated_for = 0xFF; // recalibrate next time FFT mode is entered
    }
    else
    {
      // ---------- FFT MODE ----------
      if (calibrated_for != active_adc_channel)
      {
        if (active_adc_channel == 0)
          calibrate_noise(real, imag); // mic: learn the noise
        else
          for (uint8_t b = 0; b < 16; b++) // jack: clean signal, no gate
            noise_floor[b] = 0;
        calibrated_for = active_adc_channel;
      }

      get_spectrum(real, imag, active_adc_channel, gain, mag);

      UART_Send(0xFE);
      for (uint8_t b = 0; b < 16; b++)
      {
        uint16_t m = (mag[b] > noise_floor[b]) ? (mag[b] - noise_floor[b]) : 0;
        if (active_adc_channel == 0 && m <= DEADBAND)
          m = 0;
        UART_Send(m > 252 ? 252 : (uint8_t)m);
      }
    }
  }
}
