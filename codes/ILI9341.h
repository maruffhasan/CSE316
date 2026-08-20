#ifndef ILI9341_H_
#define ILI9341_H_

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

// Pin Definitions based on your circuit
#define ILI_PORT PORTB
#define ILI_CS   PB4
#define ILI_DC   PB3
#define ILI_RST  PB2

// Function Prototypes
void ILI9341_Init(void);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_FillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ILI9341_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ILI9341_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ILI9341_WriteString(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, uint8_t size);

#endif

