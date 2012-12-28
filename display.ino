Display::Display()
  : tft_(10, 9, 8) { }

void Display::initialize() {
  tft().initR(INITR_REDTAB);
}

void Display::draw_pixel(int16_t x, int16_t y) {
  if (y < 0 || y >= kHeight)
    return;
  if (x < min_x_)
    return;
  x -= min_x_;
  if (x >= (kWidth / 2))
    return;
  uint16_t index = ((x << kLogHeight) + y);
  display_buffer_[index >> 3] |= 1 << (index & 0x7);
}

// Can't be called 'abs', that's already taken.
static inline int16_t magnitude(int16_t v) {
  return (v < 0) ? -v : v;
}

void Display::draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  int16_t xdiff = magnitude(y1 - y0);
  int16_t ydiff = magnitude(x1 - x0);
  int16_t maxdiff = max(xdiff, ydiff);
  
  if (maxdiff == 0) {
    draw_pixel(x0, y0);    
    return;
  } else if (maxdiff == 1) {
    draw_pixel(x0, y0);
    draw_pixel(x1, y1);
    return;
  }
  
  int16_t steep = xdiff > ydiff;
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }
  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }
  int16_t dx = x1 - x0;
  int16_t dy = magnitude(y1 - y0);
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

void Display::transform_line(Point<double> p0, Point<double> p1) {
  Point<double> tp0 = transform().transform(p0);
  Point<double> tp1 = transform().transform(p1);
  draw_line(tp0.x, tp0.y, tp1.x, tp1.y);
}

void Display::draw_cubic_bezier_plain(Point<double> *p) {
  int16_t last_x = static_cast<int16_t>(p[0].x);
  int16_t last_y = static_cast<int16_t>(p[0].y);
  for (uint8_t i = 1; i <= kN; i++) {
    double t = ((double) i) / kN;
    Point<double> n = (p[0] * pow(1 - t, 3)) + (p[1] * 3 * t * pow(1 - t, 2)) + (p[2] * 3 * pow(t, 2) * (1 - t)) + (p[3] * pow(t, 3));
    int16_t x = static_cast<int16_t>(n.x);
    int16_t y = static_cast<int16_t>(n.y);
    draw_line(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
  }
}

void Display::draw_cubic_bezier_faster_diffs(Point<double> *p) {
  static const double d = 1.0 / kN;

  // Calculate the four ks.  
  Point<double> k0 = p[0];
  Point<double> k1 = (p[1] - p[0]) * 3;
  Point<double> k2 = (p[2] - p[1] * 2 + p[0]) * 3;
  Point<double> k3 = p[3] - p[2] * 3 + p[1] * 3 - p[0];
  
  // Caldulate the four ds.
  Point<double> d0 = k0;
  Point<double> d1 = (((k3 * d) + k2) * d + k1) * d;
  Point<double> d2 = ((k3 * (3 * d)) + k2) * (2 * d * d);
  Point<double> d3 = k3 * (6 * d * d * d);
  
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

static inline Point<fixed> mult_3(const Point<fixed> &p) {
  return (p << 1) + p;
}

void Display::draw_cubic_bezier_fixed_diffs(Point<double> *p) {
  static const double d = 1.0 / kN;
  
  Point<fixed> p0 = p[0];
  Point<fixed> p1 = p[1];
  Point<fixed> p2 = p[2];
  Point<fixed> p3 = p[3];

  // Calculate the four ks.  
  Point<fixed> k0 = p0;
  Point<fixed> k1 = mult_3(p1 - p0);
  Point<fixed> k2 = mult_3(p2 - (p1 << 1) + p0);
  Point<fixed> k3 = p3 + mult_3(p1 - p2) - p0;
  
  // Caldulate the four ds.
  Point<fixed> d0 = k0;
  Point<fixed> d1 = ((((k3 >> kLogN) + k2) >> kLogN) + k1) >> kLogN;
  Point<fixed> d2 = ((mult_3(k3) >> kLogN) + k2) >> (kLogN * 2 - 1);
  Point<fixed> d3 = mult_3(k3) >> (3 * kLogN - 1);
  
  // Plot
  int16_t last_x = static_cast<int16_t>(p[0].x);
  int16_t last_y = static_cast<int16_t>(p[0].y);
  for (uint8_t i = 1; i <= kN; i++) {
    d0 = d0 + d1;
    d1 = d1 + d2;
    d2 = d2 + d3;
    int16_t x = d0.x.to_int16();
    int16_t y = d0.y.to_int16();
    draw_line(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
  }
}

void Display::draw_cubic_bezier_raw_diffs(Point<double> *p) {
  static const double t0 = 0.0 / kN;
  static const double t1 = 1.0 / kN;
  static const double t2 = 2.0 / kN;
  static const double t3 = 3.0 / kN;

  // Calculate the four ks.  
  Point<double> d0 = (p[0] * pow(1 - t0, 3)) + (p[1] * 3 * t0 * pow(1 - t0, 2)) + (p[2] * 3 * pow(t0, 2) * (1 - t0)) + (p[3] * pow(t0, 3));
  Point<double> d1 = (p[0] * pow(1 - t1, 3)) + (p[1] * 3 * t1 * pow(1 - t1, 2)) + (p[2] * 3 * pow(t1, 2) * (1 - t1)) + (p[3] * pow(t1, 3));
  Point<double> d2 = (p[0] * pow(1 - t2, 3)) + (p[1] * 3 * t2 * pow(1 - t2, 2)) + (p[2] * 3 * pow(t2, 2) * (1 - t2)) + (p[3] * pow(t2, 3));
  Point<double> d3 = (p[0] * pow(1 - t3, 3)) + (p[1] * 3 * t3 * pow(1 - t3, 2)) + (p[2] * 3 * pow(t3, 2) * (1 - t3)) + (p[3] * pow(t3, 3));
  
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

void Display::transform_cubic_bezier(Point<double> *p) {
  p[0] = transform().transform(p[0]);
  p[1] = transform().transform(p[1]);
  p[2] = transform().transform(p[2]);
  p[3] = transform().transform(p[3]);
  draw_cubic_bezier_fixed_diffs(p);
}

#undef TX
#undef TY

void Display::clear_buffer() {
  memset(display_buffer_, 0x00, kBufferSize);
}

void Display::set_segment(uint8_t min_x, uint8_t max_x) {
  min_x_ = min_x;
  max_x_ = max_x;
}

void Display::update_transform(double zoom, double theta, double dx, double dy) {
  Matrix &m = transform();
  m.reset();
  m.translate(-kWidth / 2.0, -kHeight / 2.0);
  m.zoom(zoom);
  m.rotate(theta);
  m.translate(kWidth / 2.0 / zoom + dx, kHeight / 2.0 / zoom + dy);  
}

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

void Display::emit_bit_pair(bool first, bool second) {
  if (first) {
    if (second) {
      SPI_WRITE_12BIT(kOnColor, kOnColor);
    } else {
      SPI_WRITE_12BIT(kOnColor, kOffColor);
    }
  } else {
    if (second) {
      SPI_WRITE_12BIT(kOffColor, kOnColor);
    } else {
      SPI_WRITE_12BIT(kOffColor, kOffColor);
    }
  }
}

// This requires the tft driver to be patched with a friend declaration for the
// Display class since this code uses its internals.
void Display::blit(uint8_t row_start, uint8_t row_end, uint8_t *data,
  uint16_t on, uint16_t off) {
  Adafruit_ST7735 &tft = this->tft();
  tft.setAddrWindow(0, row_start, kWidth - 1, row_end);
  *tft.rsport |=  tft.rspinmask;
  *tft.csport &= ~tft.cspinmask;

  uint8_t *limit = data + ((kHeight * kWidth) >> 4);
  while (data != limit) {
    uint8_t byte = *(data++);
    emit_bit_pair(byte & (1 << 0), byte & (1 << 1));
    emit_bit_pair(byte & (1 << 2), byte & (1 << 3));
    emit_bit_pair(byte & (1 << 4), byte & (1 << 5));
    emit_bit_pair(byte & (1 << 6), byte & (1 << 7));
  }
  WAIT_FOR_SPI_WRITE();

  *tft.csport |= tft.cspinmask;
}

#undef SPI_WRITE_12BIT
#undef START_SPI_WRITE
#undef WAIT_FOR_SPI_WRITE

void Display::flush(uint16_t on_color, uint16_t off_color) {
  blit(min_x_, max_x_, display_buffer_, on_color, off_color);
}

