enum Operation {
  oAbsMoveTo,
  oRelMoveTo,
  oAbsLineTo,
  oRelLineTo,
  oAbsCurveTo,
  oRelCurveTo,
  oEnd
};

#define c(v) static_cast<double>(v)
#define abs_move_to(x, y) oAbsMoveTo, c(x), c(y)
#define rel_move_to(x, y) oRelMoveTo, c(x), c(y)
#define abs_line_to(x, y) oAbsLineTo, c(x), c(y)
#define rel_line_to(x, y) oRelLineTo, c(x), c(y)
#define abs_curve_to(x0, y0, x1, y1, x2, y2) oAbsCurveTo, c(x0), c(y0), c(x1), c(y1), c(x2), c(y2)
#define rel_curve_to(x0, y0, x1, y1, x2, y2) oRelCurveTo, c(x0), c(y0), c(x1), c(y1), c(x2), c(y2)
#define end() oEnd

PROGMEM static double hand_program[] = {
  rel_move_to(129.4655,52.422114),
  rel_line_to(11.41819,0),
  rel_curve_to(2.76804,6.760496, 4.15207,13.174903, 4.15207,19.243239),
  rel_curve_to(0,6.281392, -1.38403,12.349793, -4.15207,18.205221),
  rel_line_to(-8.78322,0),
  rel_line_to(-7.98475,6.946728),
  rel_curve_to(-7.55886,4.045648, -17.30024,6.866918, -29.224166,8.463838),
  rel_line_to(-9.581695,-0.87833),
  rel_line_to(-26.509353,15.25087),
  rel_curve_to(-3.992292,2.28896, -7.186183,3.43345, -9.581696,3.43344),
  rel_curve_to(-2.129179,1e-5, -3.912431,-0.692, -5.349777,-2.07605),
  rel_curve_to(-1.490394,-1.43722, -2.235629,-3.16726, -2.235736,-5.19007),
  rel_curve_to(1.07e-4,-2.98095, 2.448754,-5.96192, 7.345961,-8.94292),
  abs_line_to(71.735789,93.064472),
  abs_line_to(39.158031,101.60815),
  rel_curve_to(-4.045502,1.06466, -6.840161,1.59699, -8.383989,1.59694),
  rel_curve_to(-2.980853,5e-5, -5.189968,-0.63874, -6.627336,-1.91633),
  rel_curve_to(-1.490372,-1.330749, -2.235617,-3.300322, -2.235736,-5.908712),
  rel_curve_to(1.19e-4,-4.471412, 3.406953,-7.691923, 10.220471,-9.661543),
  rel_line_to(33.456095,-9.581694),
  rel_line_to(-38.167087,0),
  rel_curve_to(-4.151952,6e-5, -7.265993,-0.745182, -9.342155,-2.23573),
  rel_curve_to(-2.075907,-1.490422, -3.113924,-3.726148, -3.114052,-6.707186),
  rel_curve_to(1.28e-4,-5.908636, 4.684511,-8.86299, 14.053147,-8.863068),
  abs_line_to(67.184484,59.209148),
  abs_line_to(37.481228,49.228216),
  abs_curve_to(28.751355,46.300567, 24.386355,42.707433, 24.386237,38.448811),
  rel_curve_to(1.18e-4,-2.44855, 0.612293,-4.285043, 1.836505,-5.509478),
  rel_curve_to(1.224434,-1.224211, 3.087548,-1.836375, 5.589317,-1.836482),
  rel_curve_to(2.555225,1.07e-4, 5.562805,0.42596, 9.022764,1.277559),
  rel_line_to(38.486475,9.901078),
  rel_curve_to(1.809939,-0.532217, 4.604597,-0.984691, 8.383984,-1.35741),
  rel_curve_to(-0.958113,-1.703315, -1.969513,-4.072111, -3.034203,-7.106408),
  rel_curve_to(-3.672921,-2.182399, -6.414349,-4.604435, -8.224289,-7.266131),
  rel_curve_to(-1.86304,-2.661461, -2.794592,-5.589201, -2.794661,-8.783211),
  rel_curve_to(6.9e-5,-3.087313, 1.88979,-6.840139, 5.669169,-11.258498),
  abs_line_to(80.598858,4.9128788),
  abs_line_to(96.967587,22.159928),
  abs_line_to(116.61006,36.85186),
  rel_curve_to(5.42965,5.802341, 9.7148,10.992422, 12.85544,15.570254),
  rel_move_to(-1.91633,3.832679),
  abs_curve_to(121.95986,49.06861, 116.98272,43.266364, 112.6177,38.848055),
  abs_line_to(94.652011,25.433681),
  abs_line_to(80.838401,10.661899),
  rel_curve_to(-2.129201,2.555246, -3.193834,4.977282, -3.193898,7.26612),
  rel_curve_to(6.3e-5,2.555236, 0.85177,4.924031, 2.555119,7.106419),
  rel_curve_to(1.65024,2.182613, 4.178742,4.178797, 7.585509,5.988562),
  rel_curve_to(2.927791,9.368861, 8.250951,17.247135, 15.969489,23.634843),
  rel_line_to(-2.39542,3.034204),
  abs_curve_to(96.568398,54.498231, 92.682491,50.13324, 89.70147,44.597063),
  abs_curve_to(85.922079,45.07624, 82.462026,45.768251, 79.321298,46.673097),
  abs_line_to(41.713149,36.612319),
  rel_curve_to(-3.83258,-1.064531, -6.62724,-1.596845, -8.383989,-1.596941),
  rel_curve_to(-1.596834,9.6e-5, -2.767932,0.292879, -3.513295,0.878316),
  rel_curve_to(-0.798358,0.58565, -1.19759,1.463966, -1.197707,2.634957),
  rel_curve_to(1.17e-4,2.235833, 2.741546,4.285243, 8.224297,6.148259),
  rel_line_to(34.334402,11.577883),
  rel_line_to(0,6.866882),
  rel_line_to(-39.205107,-0.958171),
  rel_line_to(-2.555118,-0.07985),
  rel_curve_to(-6.97322,7.4e-5, -10.459894,1.676869, -10.460023,5.03039),
  rel_curve_to(1.29e-4,1.650248, 0.612293,2.901191, 1.836504,3.75283),
  rel_curve_to(1.171217,0.851772, 2.981088,1.277624, 5.429629,1.27756),
  rel_line_to(5.030384,-0.07985),
  rel_line_to(39.923731,0),
  rel_line_to(1.117865,6.30795),
  rel_line_to(-36.649986,9.980932),
  rel_curve_to(-6.547368,1.756691, -9.821121,4.045649, -9.821239,6.866883),
  rel_curve_to(1.18e-4,1.384062, 0.425971,2.422078, 1.277559,3.11405),
  rel_curve_to(0.798593,0.69205, 2.049542,1.038053, 3.752837,1.038015),
  rel_curve_to(1.117964,3.8e-5, 3.38031,-0.479044, 6.787037,-1.437252),
  abs_line_to(76.526636,87.2356),
  abs_line_to(79.321298,93.064466),
  abs_line_to(51.135143,110.39136),
  rel_curve_to(-3.619648,2.18252, -5.429531,4.07225, -5.429628,5.66917),
  rel_curve_to(9.7e-5,1.06465, 0.292879,1.86312, 0.878327,2.39543),
  rel_curve_to(0.58564,0.53233, 1.490576,0.79848, 2.714819,0.79846),
  rel_curve_to(1.543806,2e-5, 4.019073,-0.98475, 7.425801,-2.95435),
  rel_line_to(27.387684,-15.96949),
  rel_line_to(10.779408,0.95817),
  rel_curve_to(10.965756,-1.650143, 20.174816,-4.391566, 27.627216,-8.224284),
  rel_line_to(8.78322,-7.345967),
  rel_line_to(6.94673,0),
  rel_curve_to(1.80989,-4.684329, 2.71482,-9.634866, 2.71482,-14.851627),
  rel_curve_to(0,-4.311694, -0.90493,-9.182383, -2.71482,-14.612086),
  rel_line_to(-10.69955,0),
  end()
};

void Drawing::draw(double *program, Display &display) {
  double x = 0;
  double y = 0;
  while (true) {
    switch (static_cast<Operation>(pgm_read_float(program++))) {
      case oAbsMoveTo: {
        x = pgm_read_float(program++);
        y = pgm_read_float(program++);
        break;
      }
      case oRelMoveTo: {
        x += pgm_read_float(program++);
        y += pgm_read_float(program++);
        break;
      }
      case oAbsLineTo: {
        float tx = pgm_read_float(program++);
        float ty = pgm_read_float(program++);
        display.transform_line(x, y, tx, ty);
        x = tx;
        y = ty;
        break;
      }
      case oRelLineTo: {
        float dx = pgm_read_float(program++);
        float dy = pgm_read_float(program++);
        display.transform_line(x, y, x + dx, y + dy);
        x += dx;
        y += dy;
        break;
      }
      case oAbsCurveTo: {
        float tx0 = pgm_read_float(program++);
        float ty0 = pgm_read_float(program++);
        float tx1 = pgm_read_float(program++);
        float ty1 = pgm_read_float(program++);
        float tx2 = pgm_read_float(program++);
        float ty2 = pgm_read_float(program++);
        display.transform_cubic_bezier(x, y, tx0, ty0, tx1, ty1, tx2, ty2);
        x = tx2;
        y = ty2;
        break;
      }
      case oRelCurveTo: {
        float dx0 = pgm_read_float(program++);
        float dy0 = pgm_read_float(program++);
        float dx1 = pgm_read_float(program++);
        float dy1 = pgm_read_float(program++);
        float dx2 = pgm_read_float(program++);
        float dy2 = pgm_read_float(program++);
        display.transform_cubic_bezier(x, y, x + dx0, y + dy0, x + dx1, y + dy1, x + dx2, y + dy2);
        x += dx2;
        y += dy2;
        break;
      }      
      case oEnd:
        return;
    }
  }
}

void Drawing::draw_hand(Display &display) {
  draw(hand_program, display);
}
