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

void Display::transform_line(Point p0, Point p1) {
  Point tp0 = transform().transform(p0);
  Point tp1 = transform().transform(p1);
  draw_line(tp0.x, tp0.y, tp1.x, tp1.y);
}

void Display::draw_cubic_bezier_plain(Point *p) {
  int16_t last_x = static_cast<int16_t>(p[0].x);
  int16_t last_y = static_cast<int16_t>(p[0].y);
  for (uint8_t i = 1; i <= kN; i++) {
    double t = ((double) i) / kN;
    Point n = (p[0] * pow(1 - t, 3)) + (p[1] * 3 * t * pow(1 - t, 2)) + (p[2] * 3 * pow(t, 2) * (1 - t)) + (p[3] * pow(t, 3));
    int16_t x = static_cast<int16_t>(n.x);
    int16_t y = static_cast<int16_t>(n.y);
    draw_line(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
  }
}

void Display::draw_cubic_bezier_faster_diffs(Point *p) {
  static const double d = 1.0 / kN;

  // Calculate the four ks.  
  Point k0 = p[0];
  Point k1 = (p[1] - p[0]) * 3;
  Point k2 = (p[2] - p[1] * 2 + p[0]) * 3;
  Point k3 = p[3] - p[2] * 3 + p[1] * 3 - p[0];
  
  // Caldulate the four ds.
  Point d0 = k0;
  Point d1 = (((k3 * d) + k2) * d + k1) * d;
  Point d2 = ((k3 * (3 * d)) + k2) * (2 * d * d);
  Point d3 = k3 * (6 * d * d * d);
  
  // Plot
  int16_t last_x = static_cast<int16_t>(p[0].x);
  int16_t last_y = static_cast<int16_t>(p[0].y);
  for (uint8_t i = 1; i <= kN; i++) {
    d0 = d0 + d1;
    d1 = d1 + d2;
    d2 = d2 + d3;
    int16_t x = static_cast<int16_t>(d0.x);
    int16_t y = static_cast<int16_t>(d0.y);
    draw_line(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
  }
}

void Display::draw_cubic_bezier_raw_diffs(Point *p) {
  static const double t0 = 0.0 / kN;
  static const double t1 = 1.0 / kN;
  static const double t2 = 2.0 / kN;
  static const double t3 = 3.0 / kN;

  // Calculate the four ks.  
  Point d0 = (p[0] * pow(1 - t0, 3)) + (p[1] * 3 * t0 * pow(1 - t0, 2)) + (p[2] * 3 * pow(t0, 2) * (1 - t0)) + (p[3] * pow(t0, 3));
  Point d1 = (p[0] * pow(1 - t1, 3)) + (p[1] * 3 * t1 * pow(1 - t1, 2)) + (p[2] * 3 * pow(t1, 2) * (1 - t1)) + (p[3] * pow(t1, 3));
  Point d2 = (p[0] * pow(1 - t2, 3)) + (p[1] * 3 * t2 * pow(1 - t2, 2)) + (p[2] * 3 * pow(t2, 2) * (1 - t2)) + (p[3] * pow(t2, 3));
  Point d3 = (p[0] * pow(1 - t3, 3)) + (p[1] * 3 * t3 * pow(1 - t3, 2)) + (p[2] * 3 * pow(t3, 2) * (1 - t3)) + (p[3] * pow(t3, 3));
  
  d3 = d3 - d2;
  d2 = d2 - d1;
  d1 = d1 - d0;
  d3 = d3 - d2;
  d2 = d2 - d1;
  d3 = d3 - d2;
  
  // Plot
  int16_t last_x = static_cast<int16_t>(p[0].x);
  int16_t last_y = static_cast<int16_t>(p[0].y);
  for (uint8_t i = 1; i <= kN; i++) {
    d0 = d0 + d1;
    d1 = d1 + d2;
    d2 = d2 + d3;
    int16_t x = static_cast<int16_t>(d0.x);
    int16_t y = static_cast<int16_t>(d0.y);
    draw_line(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
  }
}

void Display::transform_cubic_bezier(Point *p) {
  p[0] = transform().transform(p[0]);
  p[1] = transform().transform(p[1]);
  p[2] = transform().transform(p[2]);
  p[3] = transform().transform(p[3]);
  draw_cubic_bezier_faster_diffs(p);
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

