#include "ST7735R_TFT.h"

#include <avr/io.h>
#include <util/delay.h>

// Adapted from Arduino.cc TFT library:
#define DELAY 0x80
PROGMEM const unsigned char ST7735R_Init_Sequence[] =
{
  9,
  ST7735R_SWRESET, DELAY, 10,
  ST7735R_SLPOUT, DELAY, 100,
  ST7735R_GAMSET, 1, GAMSET_GC2,
  ST7735R_COLMOD, 1 | DELAY, COLMOD_WRITE_16BPP, 20,
  ST7735R_MADCTL, 1, MADCTL_MX | MADCTL_MY | MADCTL_RGB,
  ST7735R_CASET, 4, 0, 0, 0, ST7735R_WIDTH-1,
  ST7735R_RASET, 4, 0, 0, 0, ST7735R_HEIGHT-1,
  ST7735R_NORON, DELAY, 2,
  ST7735R_DISPON, DELAY, 100
};

void inline delay_ms(uint16_t ms)
{
  for (uint16_t i = 0; i < ms; i++)
    _delay_ms(1);
}

// Adapted from Arduino.cc TFT library:
void ST7735R_SendCommandList(const uint8_t *addr)
{
  uint8_t numCommands = pgm_read_byte(addr++);
  while(numCommands--)
  {
    SEND_COMMAND(pgm_read_byte(addr++));
    uint8_t numArgs = pgm_read_byte(addr++);
    uint8_t ms = numArgs & DELAY;
    numArgs &= ~DELAY;
    while(numArgs--) WRITE_SPI_SYNC(pgm_read_byte(addr++));
    if (ms) delay_ms((uint16_t)pgm_read_byte(addr++)*5);
  }
}

// Initializes the ST7735R display. Call this at startup.
void ST7735R_Begin()
{
  ST7735R_CS_DDR |= ST7735R_CS_PIN;
  ST7735R_RS_DDR |= ST7735R_RS_PIN;
  ST7735R_RST_DDR |= ST7735R_RST_PIN;
  ST7735R_LED_DDR |= ST7735R_LED_PIN;

  BEGIN_TFT();

  ST7735R_RST_PORT |= ST7735R_RST_PIN;
  _delay_ms(500);
  ST7735R_RST_PORT &= ~ST7735R_RST_PIN;
  _delay_ms(500);
  ST7735R_RST_PORT |= ST7735R_RST_PIN;
  _delay_ms(500);

  ST7735R_BEGIN_TRANSACTION();
  ST7735R_SendCommandList(ST7735R_Init_Sequence);
  // END_TFT();
  ST7735R_END_TRANSACTION();
  ST7735R_LED_PORT |= ST7735R_LED_PIN;
}

inline void swap_int(int &a, int &b)
{
  int tmp = a;
  a = b;
  b = tmp;
}

void ST7735R_Line(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
  int dx = x1 - x0;
  int dy = y1 - y0;
  int adx = abs(dx);
  int ady = abs(dy);

  uint8_t lo = LO8_RGB24(r,g,b);
  uint8_t hi = HI8_RGB24(r,g,b);

  if (ady < adx) // "Mostly horizontal": Bresenham's algorithm in the variant where x travels faster than y.
  {
    if (x1 < x0)
    {
      swap_int(x0, x1);
      swap_int(y0, y1);
      dx = -dx;
      dy = -dy;
    }
    int ysign = (dy >= 0) ? 1 : -1;

    // General strategy here is the same as with the other graphics functions:
    // interleave SPI commands with computation as much as possible, and only
    // send the minimal number of SPI CASET/RASET commands that are needed
    // needed to traverse the cursor across the line.
    ST7735R_BeginRect(x0, y0, ST7735R_WIDTH-1, ST7735R_HEIGHT-1);
    int diff = dy >> 1;
    while(x0 <= x1)
    {
      WAIT_SPI;
      WRITE_SPI_NOWAIT(hi);
      diff -= ady;
      ++x0;
      WAIT_SPI;
      WRITE_SPI_NOWAIT(lo);

      if (diff < 0)
      {
        y0 += ysign;
        diff += dx;
        WAIT_SPI;
        // The line is "mostly horizontal", so CASET and RASET end coordinates
        // are parked to bottom right of the screen, and only the start cursor
        // needs to be specified. This exploits the advantage that when the
        // y coordinate of two adjacent pixels doesn't change, we don't need
        // to change CASET/RASET commands either.
        SEND_COMMAND(ST7735R_CASET);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(x0);
        SEND_COMMAND(ST7735R_RASET);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(y0);
        SEND_COMMAND(ST7735R_RAMWR);
      }
    }
  }
  else // "Mostly vertical": Bresenham's algorithm in the variant where y travels faster than x.
  {
    if (y1 < y0)
    {
      swap_int(x0, x1);
      swap_int(y0, y1);
      dx = -dx;
      dy = -dy;
    }
    int xsign = (dx >= 0) ? 1 : -1;

    ST7735R_BeginRect(x0, y0, x0, ST7735R_HEIGHT-1);

    int diff = dx >> 1;
    while(y0 <= y1)
    {
      WAIT_SPI;
      WRITE_SPI_NOWAIT(hi);
      diff -= adx;
      ++y0;
      WAIT_SPI;
      WRITE_SPI_NOWAIT(lo);

      if (diff < 0)
      {
        x0 += xsign;
        diff += dy;
        WAIT_SPI;
        // The line is "mostly vertical", so have the CASET be a single pixel
        // window, so that the cursor traverses downwards naturally. YEND is
        // parked to bottom end of the screen. Only need to send CASET/RASET
        // commands again when the x coordinate changes. Overall the
        // "mostly vertical" path is two SPI commands slower per x coord
        // delta compared to the "mostly horizontal" path above.
        SEND_COMMAND(ST7735R_CASET);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(x0);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(x0);
        SEND_COMMAND(ST7735R_RASET);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(y0);
        SEND_COMMAND(ST7735R_RAMWR);
      }
    }
  }
  WAIT_SPI;
}

void ST7735R_Circle(int x, int y, uint8_t radius, uint8_t red, uint8_t green, uint8_t blue)
{
  // TODO: x-y clipping.
  uint8_t lo = LO8_RGB24(red, green, blue);
  uint8_t hi = HI8_RGB24(red, green, blue);

  int tx = 0;
  int ty = radius;
  int error = (5 - (radius << 2)) >> 2;

//  ST7735R_Pixel(x, y - ty, r, g, b);
  ST7735R_BeginRect(x, y - ty, ST7735R_WIDTH-1, ST7735R_HEIGHT-1);
  PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x, y + ty, r, g, b);
  CURSORY(y + ty);
  PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x - ty, y, r, g, b);
  CURSOR(x - ty, y);
  PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x + ty, y, r, g, b);
  CURSORX(x + ty);
  PUSH_PIXEL(hi, lo);

  while(tx < ty)
  {
    ++tx;
    if (error < 0)
      error += (tx << 1) + 1;
    else
    {
      --ty;
      error += ((tx - ty) << 1) + 1;
    }

    WAIT_SPI;
//  ST7735R_Pixel(x - tx, y - ty, r, g, b);
    CURSOR(x - tx, y - ty);
    PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x + tx, y - ty, r, g, b);
    CURSORX(x + tx);
    PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x + tx, y + ty, r, g, b);
    CURSORY(y + ty);
    PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x - tx, y + ty, r, g, b);
    CURSORX(x - tx);
    PUSH_PIXEL(hi, lo);

//  ST7735R_Pixel(x - ty, y - tx, r, g, b);
    CURSOR(x - ty, y - tx);
    PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x + ty, y - tx, r, g, b);
    CURSORX(x + ty);
    PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x + ty, y + tx, r, g, b);
    CURSORY(y + tx);
    PUSH_PIXEL(hi, lo);
//  ST7735R_Pixel(x - ty, y + tx, r, g, b);
    CURSORX(x - ty);
    ST7735R_PushPixel_U16(lo, hi);
  }
  WAIT_SPI;
}

void ST7735R_FilledCircle(int x, int y, uint8_t radius, uint8_t red, uint8_t green, uint8_t blue)
{
  // TODO: x-y clipping.
  uint8_t lo = LO8_RGB24(red, green, blue);
  uint8_t hi = HI8_RGB24(red, green, blue);

  int tx = 0;
  int ty = radius;
  int error = (5 - (radius << 2)) >> 2;

  ST7735R_BeginRect(x - ty, y, ST7735R_WIDTH-1, ST7735R_HEIGHT-1);

  int nty = (ty << 1) + 1;
  for(int i = 0; i < nty; ++i)
    ST7735R_PushPixel_U16(lo, hi);

  while(tx < ty)
  {
    ++tx;
    if (error < 0)
      error += (tx << 1) + 1;
    else
    {
      --ty;
      error += ((tx - ty) << 1) + 1;
    }
    WAIT_SPI;
    CURSOR(x - tx, y - ty);
    int ntx = (tx << 1) + 1;
    for(int i = 0; i < ntx; ++i)
      ST7735R_PushPixel_U16(lo, hi);
    WAIT_SPI;
    CURSORY(y + ty);
    for(int i = 0; i < ntx; ++i)
      ST7735R_PushPixel_U16(lo, hi);
    WAIT_SPI;
    int nty = (ty << 1) + 1;
    CURSOR(x - ty, y - tx);
    for(int i = 0; i < nty; ++i)
      ST7735R_PushPixel_U16(lo, hi);
    WAIT_SPI;
    CURSORY(y + tx);
    for(int i = 0; i < nty; ++i)
      ST7735R_PushPixel_U16(lo, hi);
  }
  WAIT_SPI;
}

void ST7735R_DrawMonoSprite(int x, int y, const uint8_t *addr, int width, int height,
                            uint8_t lo, uint8_t hi, uint8_t bgLo, uint8_t bgHi)
{
  ST7735R_BeginRect(x, y, x + width-1, ST7735R_HEIGHT-1);
  int nBytes = (width*height) >> 3; // Width * height must be divisible by 8!
  while(nBytes-- > 0)
  {
    uint8_t byte = pgm_read_byte(addr++);
    for(uint8_t bit = 1; bit; bit <<= 1)
    {
      if ((byte & bit) != 0) ST7735R_PushPixel_U16(lo, hi);
      else ST7735R_PushPixel_U16(bgLo, bgHi);
    }
  }
  ST7735R_EndDraw();
}

#ifdef ST7735R_MONACO_FONT
void ST7735R_DrawText(int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b,
                      uint8_t bgR, uint8_t bgG, uint8_t bgB)
{
  uint8_t ch = *text;
  uint8_t lo = LO8_RGB24(r,g,b);
  uint8_t hi = HI8_RGB24(r,g,b);
  uint8_t bgLo = LO8_RGB24(bgR,bgG,bgB);
  uint8_t bgHi = HI8_RGB24(bgR,bgG,bgB);
  while(ch)
  {
    int8_t height_adjust = pgm_read_byte(monaco_height_adjust + ch-32);
    ST7735R_DrawMonoSprite(x, y + height_adjust, monaco_font + (ch-32) * 5, 5, 8, lo, hi, bgLo, bgHi);
    ++text;
    ch = *text;
    x += 6;
  }
}
#endif


