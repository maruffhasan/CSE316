// ATmega8 @ 1 MHz : 32-sample capture -> 32-point FFT -> 16 positive bins ->
// UART
//
// Packets sent to the Arduino:
//   0xFE + 16 bytes  : FFT magnitude of bins 1..16   (bin k = k * MAX_FREQ_HZ /
//   16) 0xFF + 32 bytes  : raw waveform (when PB0 is HIGH)
//
// MAX_FREQ_HZ sets the REAL sampling rate (fs = 2 * MAX_FREQ_HZ), so the 16
// bins always span exactly 0 .. MAX_FREQ_HZ.  Use the SAME value in
// arduino.ino.

/*#define F_CPU 1000000UL
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>
#define MAX_FREQ_HZ 6000UL // <<< tunable, must match arduino.ino (MAX_FREQ)

#define N 32         // FFT size
#define UBRR_VALUE 1 // U2X, 1 MHz -> 62500 baud (exact)

// ---- sampling period and ADC clock chosen from MAX_FREQ_HZ -----------------
#define SAMPLE_PERIOD_US (1000000UL / (2UL * MAX_FREQ_HZ))

#if (SAMPLE_PERIOD_US < 40UL)
#error "MAX_FREQ_HZ too high for a 1 MHz ATmega (limit is about 12500 Hz)"
#elif (SAMPLE_PERIOD_US >= 260UL)
#define ADC_PS (1 << ADPS2) // /16 -> 208 us conversion
#elif (SAMPLE_PERIOD_US >= 130UL)
#define ADC_PS ((1 << ADPS1) | (1 << ADPS0)) // /8  -> 104 us
#elif (SAMPLE_PERIOD_US >= 70UL)
#define ADC_PS (1 << ADPS1) // /4  -> 52 us
#else
#define ADC_PS (1 << ADPS0) // /2  -> 26 us (lower accuracy)
#endif

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

// Timer1 free-runs in CTC mode at 1 MHz -> one compare match every sample
// period
static void Timer_Init(void)
{
  OCR1A = (uint16_t)(SAMPLE_PERIOD_US - 1);
  TCNT1 = 0;
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS10);
  TIFR = (1 << OCF1A);
}

static void capture(int16_t *out)
{
  TIFR = (1 << OCF1A); // start fresh
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

// In-place radix-2 FFT, N = 32.  Twiddle index into the 64-entry table is
// m*32/l.
static void compute_fft(int16_t fr[], int16_t fi[])
{
  uint8_t i, j, k, m, l, step, sh;
  int16_t t, tr, ti;

  j = 0;
  for (i = 0; i < N - 1; i++)
  { // bit reversal
    if (i < j)
    {
      t = fr[j];
      fr[j] = fr[i];
      fr[i] = t;
      t = fi[j];
      fi[j] = fi[i];
      fi[i] = t;
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
int main(void)
{
  int16_t real[N], imag[N];
  uint8_t last_pb0 = 1; // NEW: Track the previous state of the pin

  DDRB &= ~(1 << PB0);
  PORTB |= (1 << PB0); // Enable internal pull-up

  UART_Init();
  ADC_Init();
  Timer_Init();

  while (1)
  {
    capture(real);

    // Read the current state of PB0 (1 = unconnected, 0 = grounded)
    uint8_t current_pb0 = (PINB & (1 << PB0)) ? 1 : 0;

    // THE FIX: If the pin JUST got grounded, the Arduino likely suffered a voltage dip and reset.
    // We force the ATmega to stay completely silent for 2 seconds to let the Arduino safely boot up.
    if (last_pb0 == 1 && current_pb0 == 0)
    {
      for (uint8_t d = 0; d < 20; d++)
      {
        _delay_ms(100);
      }
    }
    last_pb0 = current_pb0;

    if (current_pb0 == 0)
    {
      UART_Send(0xFF); // wave header

      // NEW: Calculate the DC offset (mean) so we can center the wave
      int16_t sum = 0;
      for (uint8_t i = 0; i < N; i++)
      {
        sum += real[i];
      }
      int16_t mean = sum >> 5; // Divide by 32

      // Send the centered wave data
      for (uint8_t i = 0; i < N; i++)
      {
        int16_t v = (real[i] - mean) + 128; // Center the wave at 128 (middle of screen)
        if (v < 0)
          v = 0; // Prevent underflow glitches
        UART_Send(v > 252 ? 252 : (uint8_t)v);
      }
      _delay_ms(40);
    }
    else
    {
      int16_t sum = 0;
      for (uint8_t i = 0; i < N; i++)
      {
        sum += real[i];
        imag[i] = 0;
      }
      int16_t mean = sum >> 5;
      for (uint8_t i = 0; i < N; i++)
      { // DC removal + Hann window
        int16_t w = (127 - Sinewave[(2 * i + 16) & 63]) >> 1;
        real[i] = (int16_t)(((int32_t)(real[i] - mean) * w) >> 7);
      }
      compute_fft(real, imag);

      UART_Send(0xFE); // FFT header
      for (uint8_t i = 1; i <= 16; i++)
      { // positive bins 1..16 (16 = Nyquist)
        uint16_t mag = ((uint16_t)abs(real[i]) + (uint16_t)abs(imag[i])) >> 3;
        UART_Send(mag > 252 ? 252 : (uint8_t)mag);
      }
    }
  }
}*/
// ATmega8 @ 1 MHz : 32-sample capture -> 32-point FFT -> 16 positive bins ->
// UART
//
// Packets sent to the Arduino:
//   0xFE + 16 bytes  : FFT magnitude of bins 1..16   (bin k = k * MAX_FREQ_HZ /
//   16) 0xFF + 32 bytes  : raw waveform (when PB0 is HIGH)
//
// MAX_FREQ_HZ sets the REAL sampling rate (fs = 2 * MAX_FREQ_HZ), so the 16
// bins always span exactly 0 .. MAX_FREQ_HZ.  Use the SAME value in
// arduino.ino.

// ATmega8 @ 1 MHz : 32-sample capture -> 32-point FFT -> 16 positive bins -> UART
//
// Packets sent to the Arduino:
//   0xFE + 16 bytes  : FFT magnitude of bins 1..16   (bin k = k * MAX_FREQ_HZ / 16)
//   0xFF + 32 bytes  : raw waveform (when PB0 is HIGH)
//
// MAX_FREQ_HZ sets the REAL sampling rate (fs = 2 * MAX_FREQ_HZ), so the 16
// bins always span exactly 0 .. MAX_FREQ_HZ.  Use the SAME value in arduino.ino.
// ATmega8 @ 1 MHz : 32-sample capture -> 32-point FFT -> 16 positive bins -> UART
//
// Packets sent to the Arduino:
//   0xFE + 16 bytes  : FFT magnitude of bins 1..16   (bin k = k * MAX_FREQ_HZ / 16)
//   0xFF + 32 bytes  : raw waveform (when PB0 is HIGH)
//
// MAX_FREQ_HZ sets the REAL sampling rate (fs = 2 * MAX_FREQ_HZ), so the 16
// bins always span exactly 0 .. MAX_FREQ_HZ.  Use the SAME value in arduino.ino.
// ATmega8 @ 1 MHz : 32-sample capture -> 32-point FFT -> 16 positive bins -> UART
//
// Packets sent to the Arduino:
//   0xFE + 16 bytes  : FFT magnitude of bins 1..16   (bin k = k * MAX_FREQ_HZ / 16)
//   0xFF + 32 bytes  : raw waveform (when PB0 is HIGH)
//
// MAX_FREQ_HZ sets the REAL sampling rate (fs = 2 * MAX_FREQ_HZ), so the 16
// bins always span exactly 0 .. MAX_FREQ_HZ.  Use the SAME value in arduino.ino.
#define F_CPU 1000000UL
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>

#define MAX_FREQ_HZ 6000UL 

#define N 32         
#define UBRR_VALUE 1 

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

  for (uint8_t d = 0; d < 3; d++)
  {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
      ;
  }

  TIFR = (1 << OCF1A); 
  for (uint8_t i = 0; i < N; i++)
  {
    while (!(TIFR & (1 << OCF1A))) 
      ;
    TIFR = (1 << OCF1A);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
      ;
    out[i] = (int8_t)(ADCH - 128);
  }
}

static void compute_fft(int16_t fr[], int16_t fi[])
{
  uint8_t i, j, k, m, l, step, sh;
  int16_t t, tr, ti;

  j = 0;
  for (i = 0; i < N - 1; i++)
  { 
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

  sh = 5; 
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

int main(void)
{
  int16_t real[N], imag[N];
  uint8_t last_pb0 = 1;
  uint8_t active_adc_channel;

  // Set PB0 and PB1 as inputs
  DDRB &= ~((1 << PB0) | (1 << PB1));
  
  // CRITICAL FIX: Enable internal pull-up ONLY for PB0. 
  // We disable the pull-up for PB1 so it doesn't output leakage voltage.
  PORTB |= (1 << PB0); 
  PORTB &= ~(1 << PB1); 

  UART_Init();
  ADC_Init();
  Timer_Init();

  while (1)
  {
    // Your exact logic
    if ((PINB & (1 << PB1)) == 0)
    {
      active_adc_channel = 0;
    }
    else
    {
      active_adc_channel = 1;
    }
    
    capture(real, active_adc_channel);

    uint8_t current_pb0 = (PINB & (1 << PB0)) ? 1 : 0;

    if (last_pb0 == 1 && current_pb0 == 0)
    {
      for (uint8_t d = 0; d < 20; d++)
      {
        _delay_ms(100);
      }
    }
    last_pb0 = current_pb0;

    if (current_pb0 == 0)
    {
      UART_Send(0xFF); 

      int16_t sum = 0;
      for (uint8_t i = 0; i < N; i++)
      {
        sum += real[i];
      }
      int16_t mean = sum >> 5; 

      for (uint8_t i = 0; i < N; i++)
      {
        int16_t v = (real[i] - mean) + 128; 
        if (v < 0)
          v = 0; 
        UART_Send(v > 252 ? 252 : (uint8_t)v);
      }
      _delay_ms(40);
    }
    else
    {
      int16_t sum = 0;
      for (uint8_t i = 0; i < N; i++)
      {
        sum += real[i];
        imag[i] = 0;
      }
      int16_t mean = sum >> 5;
      for (uint8_t i = 0; i < N; i++)
      { 
        int16_t w = (127 - Sinewave[(2 * i + 16) & 63]) >> 1;
        real[i] = (int16_t)(((int32_t)(real[i] - mean) * w) >> 7);
      }
      compute_fft(real, imag);

      UART_Send(0xFE); 
      for (uint8_t i = 1; i <= 16; i++)
      { 
        uint16_t mag = ((uint16_t)abs(real[i]) + (uint16_t)abs(imag[i])) >> 3;
        UART_Send(mag > 252 ? 252 : (uint8_t)mag);
      }
    }
  }
}