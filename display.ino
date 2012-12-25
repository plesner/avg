Display::Display()
  : tft_(10, 9, 8) { }

void Display::initialize() {
  tft().initR(INITR_REDTAB);
}

void Display::draw_pixel(int16_t x, int16_t y) {
  if (y < 0 || y >= HEIGHT)
    return;
  if (x < start_row_ || x >= end_row_)
    return;
  uint16_t index = (kBufferSize * 8 - 1) - (((x - start_row_) * HEIGHT) + y);
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

#define T(v) ((int16_t) (v))
#define TX(x, y) T((x) * transform().x_.x_ + (y) * transform().x_.y_ + transform().x_.w_)
#define TY(x, y) T((x) * transform().y_.x_ + (y) * transform().y_.y_ + transform().y_.w_)

void Display::transform_cubic_bezier(double x0, double y0, double x1, double y1,
    double x2, double y2, double x3, double y3) {
  draw_cubic_bezier(TX(x0, y0), TY(x0, y0), TX(x1, y1), TY(x1, y1), TX(x2, y2), TY(x2, y2), TX(x3, y3), TY(x3, y3));
}

void Display::transform_line(double x0, double y0, double x1, double y1) {
  draw_line(TX(x0, y0), TY(x0, y0), TX(x1, y1), TY(x1, y1));
}

void Display::clear_buffer() {
  for (uint16_t i = 0; i < kBufferSize; i++)
    display_buffer_[i] = 0x00;
}

void Display::set_segment(uint8_t start_row, uint8_t end_row) {
  start_row_ = start_row;
  end_row_ = end_row;
}

void Display::update_transform(double zoom, double theta, double dx, double dy) {
  Matrix &m = transform();
  m.reset();
  m.translate(-WIDTH / 2.0, -HEIGHT / 2.0);
  m.zoom(zoom);
  m.rotate(theta);
  m.translate(WIDTH / 2.0 / zoom + dx, HEIGHT / 2.0 / zoom + dy);  
}

void Display::spi_write(uint8_t c) {
  SPDR = c;
  while (!(SPSR & _BV(SPIF)))
    ;
}

// This requires the tft driver to be patched with a friend declaration for the
// Display class since this code uses its internals.
void Display::blit(uint8_t row_start, uint8_t row_end, uint8_t *data,
  uint16_t on, uint16_t off) {
  Adafruit_ST7735 &tft = this->tft();
  tft.setAddrWindow(0, row_start, WIDTH - 1, row_end);
  *tft.rsport |=  tft.rspinmask;
  *tft.csport &= ~tft.cspinmask;
  
  uint8_t on_hi = on >> 8;
  uint8_t on_lo = on;
  uint8_t off_hi = off >> 8;
  uint8_t off_lo = off;

#define BIT(n) do { \
  if ((byte & (1 << n)) != 0) { \
    spi_write(on_hi); \
    spi_write(on_lo); \
  } else { \
    spi_write(off_hi); \
    spi_write(off_lo); \
  } \
} while (false)

  uint16_t length = (HEIGHT / 2) * WIDTH;
  for (uint16_t i = length; i > 0; i -= 8) {
    uint8_t byte = data[(i >> 3)];
    BIT(7);
    BIT(6);
    BIT(5);
    BIT(4);
    BIT(3);
    BIT(2);
    BIT(1);
    BIT(0);
  }

#undef BIT
  
  *tft.csport |= tft.cspinmask;
}

void Display::flush(uint16_t on_color, uint16_t off_color) {
  blit(start_row_, end_row_, display_buffer_, on_color, off_color);
}
