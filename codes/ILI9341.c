#include "ILI9341.h"
#include "Font5x8.h"

// Basic SPI Write
static void SPI_Write(uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
}

static void WriteCommand(uint8_t cmd) {
    ILI_PORT &= ~(1 << ILI_DC); // DC low for command
    ILI_PORT &= ~(1 << ILI_CS); // CS low
    SPI_Write(cmd);
    ILI_PORT |= (1 << ILI_CS);  // CS high
}

static void WriteData(uint8_t data) {
    ILI_PORT |= (1 << ILI_DC);  // DC high for data
    ILI_PORT &= ~(1 << ILI_CS); // CS low
    SPI_Write(data);
    ILI_PORT |= (1 << ILI_CS);  // CS high
}

#define ILI9341_SCREEN_WIDTH 320

static void SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    // This simulated panel maps CASET to app's Y and PASET to app's X
    // (confirmed by header text rotating 90 degrees when unswapped).
    // Additionally, PASET's address direction runs RIGHT-TO-LEFT on
    // screen -- address 0 sits at the physical right edge, and larger
    // addresses move left. That's the actual reason bars anchored on
    // the right and grew leftward instead of the intended left-anchor/
    // grow-right, and it's the same reason text kept coming out
    // mirrored no matter how DrawChar/WriteString were patched: those
    // were fixing the symptom in the wrong layer. Flipping the X range
    // here, once, fixes bars and text together with no per-character
    // hacks needed.
    WriteCommand(0x2A); // Column Address Set -> receives app's Y (0-239)
    WriteData(y1 >> 8); WriteData(y1 & 0xFF);
    WriteData(y2 >> 8); WriteData(y2 & 0xFF);

    uint16_t fx1 = (ILI9341_SCREEN_WIDTH - 1) - x2;
    uint16_t fx2 = (ILI9341_SCREEN_WIDTH - 1) - x1;

    WriteCommand(0x2B); // Page Address Set -> receives flipped app's X
    WriteData(fx1 >> 8); WriteData(fx1 & 0xFF);
    WriteData(fx2 >> 8); WriteData(fx2 & 0xFF);

    WriteCommand(0x2C); // Memory Write
}

void ILI9341_Init(void) {
    ILI_PORT &= ~(1 << ILI_RST); // Reset
    _delay_ms(10);
    ILI_PORT |= (1 << ILI_RST);
    _delay_ms(120);
    
    WriteCommand(0x11); // Sleep Out
    _delay_ms(120);
    WriteCommand(0x29); // Display ON
    
    // MADCTL: this simulated panel ignores MV (row/column swap) but does
    // honor the mirror bits. Text was rendering mirrored (right-to-left)
    // at the top-right instead of normal reading order at the top-left,
    // so MX=1 flips it back. Bars are unaffected since their positions
    // come from fixed coordinate math, not MADCTL.
    // MADCTL mirror-bit guessing was corrupting bar geometry too (it
    // shares SetWindow with everything else). Left at a neutral value;
    // the header text mirroring is fixed in software below instead.
    WriteCommand(0x36); WriteData(0x08); // BGR=1 only
}

void ILI9341_FillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    SetWindow(x1, y1, x2, y2);
    ILI_PORT |= (1 << ILI_DC);
    ILI_PORT &= ~(1 << ILI_CS);
    uint32_t pixels = (x2 - x1 + 1) * (y2 - y1 + 1);
    for(uint32_t i = 0; i < pixels; i++) {
        SPI_Write(color >> 8);
        SPI_Write(color & 0xFF);
    }
    ILI_PORT |= (1 << ILI_CS);
}

void ILI9341_FillScreen(uint16_t color) {
    ILI9341_FillRectangle(0, 0, 319, 239, color);
}

void ILI9341_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    ILI9341_FillRectangle(x1, y1, x2, y1, color); // Top
    ILI9341_FillRectangle(x1, y2, x2, y2, color); // Bottom
    ILI9341_FillRectangle(x1, y1, x1, y2, color); // Left
    ILI9341_FillRectangle(x2, y1, x2, y2, color); // Right
}

// Optimized for horizontal lines used in your falling dots
void ILI9341_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    if(y1 == y2) { ILI9341_FillRectangle(x1, y1, x2, y2, color); }
}
/*
void ILI9341_WriteString(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, uint8_t size) {
    // Simple placeholder for text - SimulIDE handles graphical rendering smoothly
    while(*str) {
        // Text drawing logic goes here using Font5x8
        str++;
    }
}*/
//Character Draw
void ILI9341_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    if(c < 32 || c > 126) return;

    uint16_t char_index = (c - 32) * 6;

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = pgm_read_byte(&Font5x8[char_index + i + 1]);

        for (uint8_t j = 0; j < 8; j++) {
            if (line & 0x01) {
                if (size == 1) {
                    ILI9341_FillRectangle(x + i, y + j, x + i, y + j, color);
                } else {
                    ILI9341_FillRectangle(x + i*size, y + j*size, x + i*size + size - 1, y + j*size + size - 1, color);
                }
            } else if (bg != color) {
                if (size == 1) {
                    ILI9341_FillRectangle(x + i, y + j, x + i, y + j, bg);
                } else {
                    ILI9341_FillRectangle(x + i*size, y + j*size, x + i*size + size - 1, y + j*size + size - 1, bg);
                }
            }
            line >>= 1;
        }
    }
}

//to print string
void ILI9341_WriteString(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, uint8_t size) {
    // Draw characters in the order they appear in the string. The previous
    // version walked the string backwards while still advancing cursor_x
    // forward, which placed the *last* character first -- that's what was
    // printing "MIC: PA0" as "0AP :CIM". Each glyph itself was already
    // correct; only the character sequence was reversed.
    uint16_t cursor_x = x;
    while (*str) {
        ILI9341_DrawChar(cursor_x, y, *str, fg, bg, size);
        cursor_x += 6 * size;
        if (cursor_x > 320) break;
        str++;
    }
}