#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

// How far can you zoom in? Don't make this too high or it becomes impossible to
// navigate.
static const double kMaxZoom = 8.0;

// How far can you zoom out?
static const double kMinZoom = 0.5;

// How many points are we painting per segment of a curve?
static const uint8_t kN = 4;

// Display parameters
static const uint8_t kWidth = 160;
static const uint8_t kHeight = 128;

// The colors we're using.
static const uint16_t kOnColor = ST7735_GREEN;
static const uint16_t kOffColor = ST7735_BLACK;

// A simple three-element floating point vector. Used for the perspective matrix.
class Vector {
public:
  // Creates a new empty vector.
  Vector() : x_(0), y_(0), w_(1) { }
  
  // Creates a new vector with the specified fields.
  Vector(double x, double y, double w) : x_(x), y_(y), w_(w) { }
public:
  double x_;
  double y_;
  double w_;
};

// A simple (x, y) point.
class Point {
public:
  inline Point(double _x, double _y) : x(_x), y(_y) { }
  inline Point operator*(double v) { return Point(x * v, y * v); }
  inline Point operator/(double v) { return Point(x / v, y / v); }
  inline Point operator+(Point p) { return Point(x + p.x, y + p.y); }
  inline Point operator-(Point p) { return Point(x - p.x, y - p.y); }
  double x;
  double y;
};

// A three by three matrix that can be used to transform point on the display.
class Matrix {
public:
  // Creates a new identity matrix.
  Matrix()
    : x_(1, 0, 0)
    , y_(0, 1, 0)
    , w_(0, 0, 1) { }
    
  // Creates a new matrix with the given columns.
  Matrix(const Vector &x, const Vector &y, const Vector &w)
    : x_(x), y_(y), w_(w) { }

  // Zoom in by the given factor.
  void zoom(double factor);

  // Resets this matrix to the identity.
  void reset();
  
  // Multiplies the given matrix onto this one.
  void multiply(const Matrix &that);
  
  // Moves this matrix by the given deltas.
  void translate(double dx, double dy);
  
  // Rotates this matrix by the given number of radians.
  void rotate(double theta);
  
  // Translates the given point.
  inline Point transform(Point p) {
    return Point(x_.x_ * p.x + x_.y_ * p.y + x_.w_, y_.x_ * p.x + y_.y_ * p.y + y_.w_);
  }

  // Prints this matrix on serial.
  void println();
public:
  Vector x_;
  Vector y_;
  Vector w_;
};

// Wrapper around the display. All the intersting stuff is in the implementaiton
// of this class.
class Display {
public:
  Display();

  // Sets a single pixel on the display buffer, ignoring pixels outside the
  // rendering area.
  inline void draw_pixel(int16_t x, int16_t y);

  // Draws a straight line from (x0, y0) to (x1, y1).
  inline void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
  
  // Transforms a line according to the transform matrix and renders it onto this
  // display.
  inline void transform_line(Point p0, Point p1);
  
  // Draws a cubic bezier curve from (x0, y0) to (x3, y3) with control points at
  // (x1, y1) and (x2, y2).
  inline void draw_cubic_bezier_plain(Point *points);
  inline void draw_cubic_bezier_raw_diffs(Point *points);
  inline void draw_cubic_bezier_faster_diffs(Point *points);
  
  // Transforms a cubic bezier according to the transform matrix and renders it
  // onto this display.
  inline void transform_cubic_bezier(Point *points);
  
  // Prepares the display for drawing. Don't call any of the draw methods before
  // you've called this.
  void initialize();

  // Sets which segment is being rendered into the display buffer.
  void set_segment(uint8_t start_row, uint8_t end_row);
  
  // Resets the display buffer.
  void clear_buffer();
  
  // Renders the display buffer onto the display.
  void flush(uint16_t on_color, uint16_t off_color);
  
  // Updates the display transformation with the given parameters.
  void update_transform(double zoom, double theta, double dx, double dy);
  
private:
  // Writes a bitmap to the display over hardware SPI.
  void blit(uint8_t row_start, uint8_t row_end, uint8_t *data, uint16_t on,
    uint16_t off);

  Adafruit_ST7735 tft_;
  
  Adafruit_ST7735 &tft() { return tft_; }
  
  static const uint16_t kBufferSize = ((kWidth * kHeight) >> 4) + 1;
  
  // The boolean display buffer we're rendering on.
  uint8_t display_buffer_[kBufferSize];
  
  uint8_t start_row_;
  
  uint8_t end_row_;
  
  // The transformation matrix that implements display control.
  Matrix transform_;
  
  Matrix &transform() { return transform_ ; }
};

class Drawing {
public:
  static void draw(double *program, Display &display);
  static void draw_hand(Display &display);
};

class Main {
public:
  // Does basic initialization.
  void setup();
  
  // Renders another step.
  void loop();
  
  // Renders the drawing on the specified segment of the display.
  void draw_segment(uint16_t start_row, uint16_t end_row, uint16_t *clear_time,
    uint16_t *draw_time, uint16_t *blit_time);
  
  // Reads navigation state from the analog pins and updates the transform matrix
  // in the display appropriately.
  void navigate();
private:
  Display display_;
  Display &display() { return display_; }
};

// The singleton Main instance.
static Main main;

void Main::setup() {
  Serial.begin(9600);
  display().initialize();
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}

void Main::draw_segment(uint16_t start_row, uint16_t end_row, uint16_t *clear_time,
    uint16_t *draw_time, uint16_t *blit_time) {
  display().set_segment(start_row, end_row);
  uint16_t start = millis();
  display().clear_buffer();
  uint16_t end = millis();
  *clear_time += end - start;
  start = end;
  Drawing::draw_hand(display());
  end = millis();
  *draw_time += end - start;
  start = end;
  display().flush(ST7735_GREEN, ST7735_BLACK);
  end = millis();  
  *blit_time += end - start;
}

void Main::navigate() {
  static const double kMax = 1024.0;
  static const double kHalf = kMax / 2.0;

  // Zoom
  uint16_t raw_zoom = analogRead(A3);
  double zoom;
  if (raw_zoom < kHalf) {
    double ratio = raw_zoom / kHalf;
    zoom = ratio + (1 - ratio) * kMaxZoom;
  } else {
    double ratio = (raw_zoom - kHalf) / kHalf;
    zoom = ratio * kMinZoom + (1 - ratio);
  }
  
  // Pan
  uint16_t raw_dx = analogRead(A0);
  double dx = ((raw_dx - kHalf) / kHalf) * kWidth;
  uint16_t raw_dy = analogRead(A1);
  double dy = ((raw_dy - kHalf) / kHalf) * kHeight;

  // Rotation
  uint16_t rotation = analogRead(A2);
  double theta = 3.141592645 * (rotation - kHalf) / kHalf;

  // Update the display.
  display().update_transform(zoom, theta, dx, dy);
}

void Main::loop() {
  uint16_t clear_time = 0;
  uint16_t draw_time = 0;
  uint16_t blit_time = 0;
  uint16_t start = millis();
  draw_segment(0, kWidth / 2, &clear_time, &draw_time, &blit_time);
  draw_segment(kWidth / 2, kWidth, &clear_time, &draw_time, &blit_time);
  uint16_t end = millis();
  Serial.print("clear: ");
  Serial.println(clear_time, DEC);
  Serial.print("draw: ");
  Serial.println(draw_time, DEC);
  Serial.print("blit: ");
  Serial.println(blit_time, DEC);
  Serial.print("total: ");
  Serial.println(end - start, DEC);
  navigate();
}

void setup() {
  main.setup();
}

void loop() {
  main.loop();
}
