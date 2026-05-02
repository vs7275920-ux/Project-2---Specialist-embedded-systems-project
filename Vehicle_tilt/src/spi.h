#include <stm32l432xx.h>
uint16_t transferSPI16(SPI_TypeDef *spi, uint16_t data);
uint8_t transferSPI8(SPI_TypeDef *spi, uint8_t data);
void initSPI(SPI_TypeDef *spi);
uint8_t spi_exchange(SPI_TypeDef *spi,uint8_t d_out[], uint32_t d_out_len, uint8_t d_in[], uint32_t d_in_len);
/*LCD*/
void init_display(void);
uint16_t RGBToWord(uint16_t R, uint16_t G, uint16_t B);
void putPixel(uint16_t x, uint16_t y, uint16_t colour);
void putImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *Image, int hOrientation, int vOrientation);
void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t Colour);
void fillRectangle(uint16_t x,uint16_t y,uint16_t width, uint16_t height, uint16_t colour);
void printText(const char *Text,uint16_t x, uint16_t y, uint16_t ForeColour, uint16_t BackColour);
void printTextX2(const char *Text, uint16_t x, uint16_t y, uint16_t ForeColour, uint16_t BackColour);
void drawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t Colour);
void drawCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t Colour);
void fillCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t Colour);
