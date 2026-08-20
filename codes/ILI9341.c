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

static void SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    WriteCommand(0x2A); // Column Address Set
    WriteData(x1 >> 8); WriteData(x1 & 0xFF);
    WriteData(x2 >> 8); WriteData(x2 & 0xFF);
    
    WriteCommand(0x2B); // Page Address Set
    WriteData(y1 >> 8); WriteData(y1 & 0xFF);
    WriteData(y2 >> 8); WriteData(y2 & 0xFF);
    
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
    
    // Set landscape rotation (optional, depends on SimulIDE setup)
    WriteCommand(0x36); WriteData(0x48); 
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

void ILI9341_WriteString(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, uint8_t size) {
    // Simple placeholder for text - SimulIDE handles graphical rendering smoothly
    while(*str) {
        // Text drawing logic goes here using Font5x8
        str++;
    }
}

