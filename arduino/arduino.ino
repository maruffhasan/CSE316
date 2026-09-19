#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8
#define TFT_ROTATION 3

// ======================= TUNABLE PARAMETERS =======================
#define MAX_FREQ     10000UL   // Hz. Must be the SAME as MAX_FREQ_HZ in atmega.c
#define HEIGHT_GAIN  4        // bar height gain
#define WAVE_GAIN    1        // waveform vertical gain
#define FREQ_CAL     1.0f     // trim if a known tone reads off (ATmega RC clock error)
#define MIN_PEAK     4        // ignore peaks below this amplitude
#define WAVE_HYST    4
#define SEG_H        6        // segment height in pixels
#define SEG_GAP      1        // gap between segments (0 = solid bar)
#define FALL_SEGS    255      // max segments a bar may drop per frame (255 = instant)
// ==================================================================

#define NUM_BINS     16       // positive FFT bins 1..16
#define NUM_SAMPLES  32
#define STATUS_H     12
#define FREQ_W       64

// Real sampling rate set by the ATmega:  fs = 1e6 / period,  bin k = k * fs / 32
#define SAMPLE_PERIOD_US (1000000UL / (2UL * MAX_FREQ))
#define SAMPLE_RATE      ((1000000.0f / (float)SAMPLE_PERIOD_US) * FREQ_CAL)
#define BIN_WIDTH        (SAMPLE_RATE / NUM_SAMPLES)

// Hardware SPI (pins 11/13 on Uno/Nano) -> much faster than the software-SPI constructor
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

int amp[NUM_BINS];
int wave[NUM_SAMPLES];
int prevWaveY[NUM_SAMPLES];
int prevSeg[NUM_BINS] = {0};
uint16_t segColor[64];

int scrW, scrH, barAreaH, slotW, barW, numSegs;
bool waveDrawn = false;
bool gotFirstPacket = false;
int lastMode = 0;
int lastFreq = -2;

void showStatus(const char *msg, uint16_t color) {
  tft.fillRect(0, 0, scrW - FREQ_W, STATUS_H, ILI9341_BLACK);
  tft.setTextSize(1); tft.setTextColor(color);
  tft.setCursor(2, 2); tft.print(msg);
}

void showFreq(int hz) {
  if (hz == lastFreq) return;
  lastFreq = hz;
  char buf[12];
  if (hz < 0) strcpy(buf, "--- Hz"); else snprintf(buf, sizeof(buf), "%d Hz", hz);
  tft.fillRect(scrW - FREQ_W, 0, FREQ_W, STATUS_H, ILI9341_BLACK);
  tft.setTextSize(1); tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(scrW - 2 - 6 * strlen(buf), 2);
  tft.print(buf);
}

void switchMode(int m) {
  if (m == lastMode) return;
  tft.fillRect(0, STATUS_H, scrW, barAreaH, ILI9341_BLACK);
  for (int b = 0; b < NUM_BINS; b++) prevSeg[b] = 0;
  waveDrawn = false;
  lastMode = m;
}

void packetOk() {
  if (!gotFirstPacket) {
    char buf[32];
    snprintf(buf, sizeof(buf), "0-%lu Hz  (%d Hz/bar)", MAX_FREQ, (int)(BIN_WIDTH + 0.5f));
    showStatus(buf, ILI9341_YELLOW);
    gotFirstPacket = true;
  }
}

// ---------------------------- spectrum ----------------------------
inline int segTop(int s) { return scrH - (s + 1) * SEG_H + SEG_GAP; }

// Only the segments that changed are touched: grow = draw new ones, shrink = one black rect
void drawBar(int b, int n) {
  int o = prevSeg[b];
  if (n == o) return;
  int x = b * slotW + (slotW - barW) / 2;
  if (n < o) {
    if (o - n > FALL_SEGS) n = o - FALL_SEGS;
    tft.fillRect(x, scrH - o * SEG_H, barW, (o - n) * SEG_H, ILI9341_BLACK);
  } else {
    for (int s = o; s < n; s++)
      tft.fillRect(x, segTop(s), barW, SEG_H - SEG_GAP, segColor[s]);
  }
  prevSeg[b] = n;
}

int findPeakFreq() {
  int best = -1, bestAmp = MIN_PEAK - 1;
  for (int i = 0; i < NUM_BINS; i++)
    if (amp[i] > bestAmp) { bestAmp = amp[i]; best = i; }
  if (best < 0) return -1;
  float pos = best + 1;                       // amp[i] is FFT bin i+1
  if (best > 0 && best < NUM_BINS - 1) {      // parabolic interpolation
    float a = amp[best - 1], b = amp[best], c = amp[best + 1];
    float d = a - 2 * b + c;
    if (d < 0) pos += 0.5f * (a - c) / d;
  }
  return (int)(pos * BIN_WIDTH + 0.5f);
}

void processData() {
  packetOk(); switchMode(1);
  for (int b = 0; b < NUM_BINS; b++) {
    int n = ((int)(amp[b] * HEIGHT_GAIN) + SEG_H / 2) / SEG_H;
    drawBar(b, constrain(n, 0, numSegs));
  }
  showFreq(findPeakFreq());
}

// ---------------------------- waveform ----------------------------
int waveFreq() {
  long s = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) s += wave[i];
  int mid = s / NUM_SAMPLES;
  float first = 0, last = 0;
  int n = 0; bool low = false;
  for (int i = 1; i < NUM_SAMPLES; i++) {
    if (wave[i] < mid - WAVE_HYST) low = true;
    else if (low && wave[i] >= mid) {
      float pos = (i - 1) + (float)(mid - wave[i - 1]) / (float)(wave[i] - wave[i - 1]);
      if (n == 0) first = pos;
      last = pos; n++; low = false;
    }
  }
  if (n < 2 || last <= first) return -1;
  return (int)((n - 1) * SAMPLE_RATE / (last - first) + 0.5f);
}

int waveX(int i) { return (int)((long)i * (scrW - 1) / (NUM_SAMPLES - 1)); }

int waveY(int v) {
  long d = (long)((v - 128) * WAVE_GAIN);
  if (d > 127) d = 127;
  if (d < -127) d = -127;
  return scrH - 1 - (int)((d + 128) * (long)(barAreaH - 1) / 255);
}

void drawWave() {
  packetOk(); switchMode(2);
  int newY[NUM_SAMPLES];
  for (int i = 0; i < NUM_SAMPLES; i++) newY[i] = waveY(wave[i]);
  if (waveDrawn)
    for (int i = 1; i < NUM_SAMPLES; i++)
      tft.drawLine(waveX(i - 1), prevWaveY[i - 1], waveX(i), prevWaveY[i], ILI9341_BLACK);
  for (int i = 1; i < NUM_SAMPLES; i++)
    tft.drawLine(waveX(i - 1), newY[i - 1], waveX(i), newY[i], ILI9341_GREEN);
  for (int i = 0; i < NUM_SAMPLES; i++) prevWaveY[i] = newY[i];
  waveDrawn = true;
  showFreq(waveFreq());
}

// Packet: 0xFE + 16 bytes (FFT bins)  or  0xFF + 32 bytes (wave)
// All waiting bytes are consumed first and only the NEWEST packet is drawn,
// so the display never lags behind the signal.
void readData() {
  static uint8_t buf[NUM_SAMPLES], n = 0, need = 0, hdr = 0;
  uint8_t got = 0;                       // 0 none, 1 FFT, 2 wave
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (c >= 0xFE) { hdr = c; need = (c == 0xFE) ? NUM_BINS : NUM_SAMPLES; n = 0; continue; }
    if (!need) continue;
    buf[n++] = c;
    if (n == need) {
      if (hdr == 0xFF) { for (int i = 0; i < NUM_SAMPLES; i++) wave[i] = buf[i]; got = 2; }
      else             { for (int i = 0; i < NUM_BINS; i++)    amp[i]  = buf[i]; got = 1; }
      need = 0;
    }
  }
  if (got == 1) processData();
  else if (got == 2) drawWave();
}

uint16_t gradientColor(int s) {          // green (bottom) -> yellow -> red (top)
  long f = (long)s * 510L / (numSegs > 1 ? numSegs - 1 : 1);
  uint8_t r, g;
  if (f < 255) { r = f; g = 255; } else { r = 255; g = 510 - f; }
  return tft.color565(r, g, 0);
}

void setup() {
  Serial.begin(62500);
  tft.begin();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(ILI9341_BLACK);
  scrW = tft.width(); scrH = tft.height();
  barAreaH = scrH - STATUS_H;
  slotW = scrW / NUM_BINS; barW = slotW - 2;
  numSegs = barAreaH / SEG_H;
  if (numSegs > 64) numSegs = 64;
  for (int s = 0; s < numSegs; s++) segColor[s] = gradientColor(s);

  showStatus("Waiting for data...", ILI9341_YELLOW);
  showFreq(-1);
}

void loop() { readData(); }
