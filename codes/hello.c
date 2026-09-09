#define F_CPU 8000000UL 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h> // Required to store fonts in Flash memory

#define RST  PB2
#define DC   PB3
#define CS   PB4
#define MOSI PB5
#define SCK  PB7

#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED   0xF800

// Store fonts in Flash memory to prevent AVR SRAM corruption
const uint8_t font_H[5] PROGMEM = {0x7F, 0x08, 0x08, 0x08, 0x7F};
const uint8_t font_E[5] PROGMEM = {0x7F, 0x49, 0x49, 0x49, 0x41};
const uint8_t font_L[5] PROGMEM = {0x7F, 0x40, 0x40, 0x40, 0x40};
const uint8_t font_O[5] PROGMEM = {0x3E, 0x41, 0x41, 0x41, 0x3E};
const uint8_t font_W[5] PROGMEM = {0x3F, 0x40, 0x38, 0x40, 0x3F};
const uint8_t font_R[5] PROGMEM = {0x7F, 0x09, 0x19, 0x29, 0x46};
const uint8_t font_D[5] PROGMEM = {0x7F, 0x41, 0x41, 0x22, 0x1C};
const uint8_t font_space[5] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t font_block[5] PROGMEM = {0x7F, 0x7F, 0x7F, 0x7F, 0x7F}; 

void SPI_Init(void) {
    DDRB |= (1<<MOSI) | (1<<SCK) | (1<<CS) | (1<<RST) | (1<<DC);
    SPCR = (1<<SPE) | (1<<MSTR);
}

void SPI_Transmit(uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
}

void ILI9341_Command(uint8_t cmd) {
    PORTB &= ~(1<<DC); 
    PORTB &= ~(1<<CS); 
    SPI_Transmit(cmd);
    PORTB |= (1<<CS);  
}

void ILI9341_Data(uint8_t data) {
    PORTB |= (1<<DC);  
    PORTB &= ~(1<<CS); 
    SPI_Transmit(data);
    PORTB |= (1<<CS);  
}

void ILI9341_Init(void) {
    PORTB |= (1<<RST);
    _delay_ms(5);
    PORTB &= ~(1<<RST);
    _delay_ms(15);      
    PORTB |= (1<<RST);  
    _delay_ms(120);     

    ILI9341_Command(0x11); 
    _delay_ms(120);
    
    ILI9341_Command(0x3A); 
    ILI9341_Data(0x55);    
    
    // Rotate to Landscape Mode to fit wider text
    ILI9341_Command(0x36); 
    ILI9341_Data(0x28);    

    ILI9341_Command(0x29); 
}

void ILI9341_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    ILI9341_Command(0x2A); 
    ILI9341_Data(x0 >> 8); ILI9341_Data(x0 & 0xFF);
    ILI9341_Data(x1 >> 8); ILI9341_Data(x1 & 0xFF);

    ILI9341_Command(0x2B); 
    ILI9341_Data(y0 >> 8); ILI9341_Data(y0 & 0xFF);
    ILI9341_Data(y1 >> 8); ILI9341_Data(y1 & 0xFF);

    ILI9341_Command(0x2C); 
}

void ILI9341_FillScreen(uint16_t color) {
    // Window expanded for landscape dimensions (320x240)
    ILI9341_SetWindow(0, 0, 319, 239);
    PORTB |= (1<<DC);  
    PORTB &= ~(1<<CS); 
    for(uint32_t i = 0; i < 76800; i++) {
        SPI_Transmit(color >> 8);
        SPI_Transmit(color & 0xFF);
    }
    PORTB |= (1<<CS); 
}

const uint8_t* get_font_bitmap(char c) {
    switch(c) {
        case 'H': return font_H;
        case 'E': return font_E;
        case 'L': return font_L;
        case 'O': return font_O;
        case 'W': return font_W;
        case 'R': return font_R;
        case 'D': return font_D;
        case ' ': return font_space;
        default: return font_block; 
    }
}

void ILI9341_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    const uint8_t *bitmap = get_font_bitmap(c);
    
    ILI9341_SetWindow(x, y, x + (5 * size) - 1, y + (8 * size) - 1);
    
    PORTB |= (1<<DC); 
    PORTB &= ~(1<<CS);
    
    for (int row = 0; row < 8 * size; row++) {
        for (int col = 0; col < 5 * size; col++) {
            // Retrieve byte safely from Flash memory
            uint8_t font_byte = pgm_read_byte(&bitmap[col / size]);
            
            if (font_byte & (1 << (row / size))) {
                SPI_Transmit(color >> 8); 
                SPI_Transmit(color & 0xFF);
            } else {
                SPI_Transmit(bg >> 8);    
                SPI_Transmit(bg & 0xFF);
            }
        }
    }
    PORTB |= (1<<CS);
}

void ILI9341_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t size) {
    int cursor_x = x;
    while (*str) {
        ILI9341_DrawChar(cursor_x, y, *str, color, bg, size);
        cursor_x += (5 * size) + size; 
        str++;
    }
}

int main(void) {
    SPI_Init();
    ILI9341_Init();
    
    ILI9341_FillScreen(BLACK);

    // Text will now comfortably fit across the 320-pixel landscape width
    ILI9341_DrawString(10, 60, "HELLO WORLD", WHITE, BLACK, 4);

    while(1) {
    }
    return 0;
}