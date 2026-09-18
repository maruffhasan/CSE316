
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  tft.begin();

  tft.setRotation(1);   // Rotate 90 degrees

  tft.fillScreen(ILI9341_BLACK);

  tft.setCursor(50, 120);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.println("Hello World!");
}