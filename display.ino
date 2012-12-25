Display::Display()
  : tft_(10, 9, 8) { }

void Display::initialize() {
  tft().initR(INITR_REDTAB);
}

void Display::draw_pixel(int16_t x, int16_t y) {
  if (y < 0 || y >= kHeight)
    return;
  if (x < start_row_ || x >= end_row_)
    return;
  uint16_t index = (kBufferSize * 8 - 1) - (((x - start_row_) * kHeight) + y);
  display_buffer_[index >> 3] |= 1 << (index & 0x7);
}

void Display::draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      draw_pixel(y0, x0);
    } else {
      draw_pixel(x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void Display::draw_cubic_bezier(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
    int16_t y2, int16_t x3, int16_t y3) {
  int16_t last_x = x0;
  int16_t last_y = y0; 
  for (uint8_t i = 1; i <= kN; i++) {
    double t = ((double) i) / kN;
    int16_t x = (int16_t) (x0 * pow(1 - t, 3) + 3 * x1 * t * pow(1 - t, 2) + 3 * x2 * pow(t, 2) * (1 - t) + x3 * pow(t, 3));
    int16_t y = (int16_t) (y0 * pow(1 - t, 3) + 3 * y1 * t * pow(1 - t, 2) + 3 * y2 * pow(t, 2) * (1 - t) + y3 * pow(t, 3));    
    draw_line(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
  }
}

#define TX(x, y) static_cast<int16_t>(transform().target_x(x, y))
#define TY(x, y) static_cast<int16_t>(transform().target_y(x, y))

void Display::transform_cubic_bezier(double x0, double y0, double x1, double y1,
    double x2, double y2, double x3, double y3) {
  draw_cubic_bezier(TX(x0, y0), TY(x0, y0), TX(x1, y1), TY(x1, y1), TX(x2, y2), TY(x2, y2), TX(x3, y3), TY(x3, y3));
}

void Display::transform_line(double x0, double y0, double x1, double y1) {
  draw_line(TX(x0, y0), TY(x0, y0), TX(x1, y1), TY(x1, y1));
}

#undef TX
#undef TY

void Display::clear_buffer() {
  memset(display_buffer_, 0x00, kBufferSize);
}

void Display::set_segment(uint8_t start_row, uint8_t end_row) {
  start_row_ = start_row;
  end_row_ = end_row;
}

void Display::update_transform(double zoom, double theta, double dx, double dy) {
  Matrix &m = transform();
  m.reset();
  m.translate(-kWidth / 2.0, -kHeight / 2.0);
  m.zoom(zoom);
  m.rotate(theta);
  m.translate(kWidth / 2.0 / zoom + dx, kHeight / 2.0 / zoom + dy);  
}

// This requires the tft driver to be patched with a friend declaration for the
// Display class since this code uses its internals.
void Display::blit(uint8_t row_start, uint8_t row_end, uint8_t *data,
  uint16_t on, uint16_t off) {
  Adafruit_ST7735 &tft = this->tft();
  tft.setAddrWindow(0, row_start, kWidth - 1, row_end);
  *tft.rsport |=  tft.rspinmask;
  *tft.csport &= ~tft.cspinmask;

// These are macros because whatever the compiler might think they absolutely
// need to be inlined.

// Waits for a fast spi write to complete.
#define WAIT_FOR_SPI_WRITE() do { \
  while (!(SPSR & _BV(SPIF))) \
    ; \
} while (false)

// Starts a fast spi write, possibly waiting for the previous one to complete.
#define START_SPI_WRITE(c) do { \
  WAIT_FOR_SPI_WRITE(); \
  SPDR = c; \
} while (false)

#define SPI_WRITE_12BIT(high, low) do { \
  START_SPI_WRITE((high >> 4) & 0xFF); \
  START_SPI_WRITE(((high & 0xF) << 4) | ((low >> 8) & 0xF)); \
  START_SPI_WRITE(low & 0xFF); \
} while (false)

#define EMIT_BIT_PAIR(t, b) do { \
  if ((byte & (1 << t)) != 0) { \
    if ((byte & (1 << b)) != 0) { \
      SPI_WRITE_12BIT(kOnColor, kOnColor); \
    } else { \
      SPI_WRITE_12BIT(kOnColor, kOffColor); \
    } \
  } else { \
    if ((byte & (1 << b)) != 0) { \
      SPI_WRITE_12BIT(kOffColor, kOnColor); \
    } else { \
      SPI_WRITE_12BIT(kOffColor, kOffColor); \
    } \
  } \
} while (false)

  uint16_t length = (kHeight / 2) * kWidth;
  for (uint16_t i = length >> 3; i > 0; i--) {
    uint8_t byte = data[i];
    EMIT_BIT_PAIR(7, 6);
    EMIT_BIT_PAIR(5, 4);
    EMIT_BIT_PAIR(3, 2);
    EMIT_BIT_PAIR(1, 0);
  }
  WAIT_FOR_SPI_WRITE();

#undef EMIT_BIT_PAIR
#undef SPI_WRITE_12BIT
#undef SPI_WRITE
#undef WAIT_FOR_SPI

  *tft.csport |= tft.cspinmask;
}

void Display::flush(uint16_t on_color, uint16_t off_color) {
  blit(start_row_, end_row_, display_buffer_, on_color, off_color);
}

