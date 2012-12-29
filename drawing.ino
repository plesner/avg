enum Operation {
  oAbsMoveTo,
  oAbsLineTo,
  oAbsCurveTo,
  oEnd
};

#define K0(P0)             (P0)
#define K1(P0, P1)         ((P1 - P0) * 3.0)
#define K2(P0, P1, P2)     ((P2 - (P1 * 2.0) + P0) * 3.0)
#define K3(P0, P1, P2, P3) (P3 - (P2 * 3.0) + (P1 * 3.0) - P0)
#define D0(P0)             K0(P0)
#define D1(P0, P1, P2, P3) ((((K3(P0, P1, P2, P3) / kN) + K2(P0, P1, P2)) / kN + K1(P0, P1)) / kN)
#define D2(P0, P1, P2, P3) ((((3.0 * K3(P0, P1, P2, P3) / kN) + K2(P0, P1, P2)) * 2.0) / kN)
#define D3(P0, P1, P2, P3) (6.0 * K3(P0, P1, P2, P3) / kN)

#define F(v) ENCODE_FIXED(static_cast<double>(v))

#ifdef PRECOMPUTED
#define curve_to(x0, y0, x1, y1, x2, y2, x3, y3) \
  oAbsCurveTo, \
  F(D0(x0)),             F(D0(y0)), \
  F(D1(x0, x1, x2, x3)), F(D1(y0, y1, y2, y3)), \
  F(D2(x0, x1, x2, x3)), F(D2(y0, y1, y2, y3)), \
  F(D3(x0, x1, x2, x3)), F(D3(y0, y1, y2, y3)), \
  F(x3), F(y3)
#else
#define curve_to(x0, y0, x1, y1, x2, y2, x3, y3) oAbsCurveTo, F(x1), F(y1), F(x2), F(y2), F(x3), F(y3)
#endif

#define move_to(x0, y0, x1, y1) oAbsMoveTo, F(x1), F(y1)
#define line_to(x0, y0, x1, y1) oAbsLineTo, F(x1), F(y1)
#define end() oEnd
#define e(V) static_cast<double>(V)

PROGMEM static int32_t australia_program[] = {
  move_to(e(0.0), e(0.0), e(47.087846), e(2.5463249)),
  line_to(e(47.087846), e(2.5463249), e(46.343521), e(2.6951896)),
  line_to(e(46.343521), e(2.6951896), e(47.087846), e(2.5463249)),
  move_to(e(47.087846), e(2.5463249), e(47.395843), e(3.0083233)),
  curve_to(e(47.395843), e(3.0083233), e(46.92731), e(3.8001528), e(46.611549), e(2.6555955), e(47.395843), e(3.0083233)),
  move_to(e(47.395843), e(3.0083233), e(47.241845), e(4.702312)),
  curve_to(e(47.241845), e(4.702312), e(45.508809), e(4.729542), e(47.173645), e(2.5769775), e(47.241845), e(4.702312)),
  move_to(e(47.241845), e(4.702312), e(45.393854), e(3.9323162)),
  curve_to(e(45.393854), e(3.9323162), e(51.350472), e(9.0086656), e(48.645609), e(17.456699), e(51.09631), e(24.054064)),
  curve_to(e(51.09631), e(24.054064), e(50.659496), e(28.806648), e(58.695716), e(31.219331), e(58.371226), e(25.953604)),
  curve_to(e(58.371226), e(25.953604), e(53.989346), e(23.364015), e(62.983545), e(28.109103), e(63.565748), e(23.644203)),
  curve_to(e(63.565748), e(23.644203), e(65.833454), e(21.035759), e(73.602895), e(20.224221), e(69.60156), e(15.762854)),
  curve_to(e(69.60156), e(15.762854), e(70.135513), e(12.627537), e(63.342108), e(10.250343), e(67.76057), e(7.6386495)),
  curve_to(e(67.76057), e(7.6386495), e(68.439078), e(10.751405), e(70.824289), e(8.5260166), e(69.418336), e(7.3247291)),
  curve_to(e(69.418336), e(7.3247291), e(73.556782), e(10.187184), e(80.027256), e(6.2629764), e(81.848829), e(4.5310777)),
  curve_to(e(81.848829), e(4.5310777), e(86.276336), e(6.2131526), e(78.259538), e(7.1066298), e(83.067259), e(8.8271793)),
  curve_to(e(83.067259), e(8.8271793), e(90.123834), e(6.0032598), e(90.638536), e(16.948223), e(93.978449), e(18.098022)),
  curve_to(e(93.978449), e(18.098022), e(97.998026), e(18.960633), e(100.05466), e(12.236031), e(103.66533), e(15.277375)),
  curve_to(e(103.66533), e(15.277375), e(104.72355), e(15.216245), e(104.58515), e(17.181383), e(106.92705), e(16.924243)),
  curve_to(e(106.92705), e(16.924243), e(107.86717), e(20.236639), e(111.66162), e(18.938809), e(109.51067), e(22.487984)),
  curve_to(e(109.51067), e(22.487984), e(110.80109), e(23.818361), e(115.92372), e(21.788053), e(112.02114), e(25.700513)),
  curve_to(e(112.02114), e(25.700513), e(114.06478), e(29.495747), e(113.82049), e(20.217064), e(116.24983), e(25.599901)),
  curve_to(e(116.24983), e(25.599901), e(118.33588), e(29.410172), e(117.29246), e(32.997168), e(122.14563), e(36.028327)),
  curve_to(e(122.14563), e(36.028327), e(126.84598), e(37.635956), e(133.07424), e(39.836277), e(136.48865), e(40.725773)),
  curve_to(e(136.48865), e(40.725773), e(138.74177), e(41.864756), e(143.05637), e(47.570596), e(143.84933), e(46.405474)),
  curve_to(e(143.84933), e(46.405474), e(144.04824), e(44.035435), e(145.92563), e(52.184251), e(145.23199), e(54.360259)),
  curve_to(e(145.23199), e(54.360259), e(144.91906), e(57.35323), e(139.67455), e(64.227301), e(144.90456), e(59.475637)),
  curve_to(e(144.90456), e(59.475637), e(142.52057), e(64.055559), e(144.59324), e(61.60194), e(146.57126), e(59.217993)),
  curve_to(e(146.57126), e(59.217993), e(141.77676), e(66.01965), e(138.46783), e(73.375038), e(135.02375), e(80.885203)),
  curve_to(e(135.02375), e(80.885203), e(133.4591), e(85.688284), e(138.61277), e(91.80358), e(130.52134), e(92.207156)),
  curve_to(e(130.52134), e(92.207156), e(124.02058), e(93.206928), e(120.3391), e(84.423293), e(113.2531), e(86.896505)),
  curve_to(e(113.2531), e(86.896505), e(107.72695), e(85.41336), e(104.49885), e(78.948849), e(97.824131), e(79.872973)),
  curve_to(e(97.824131), e(79.872973), e(91.822375), e(76.327486), e(83.395076), e(77.478865), e(78.394587), e(82.098098)),
  curve_to(e(78.394587), e(82.098098), e(77.036927), e(84.831337), e(75.25349), e(87.290559), e(74.556517), e(90.203606)),
  curve_to(e(74.556517), e(90.203606), e(72.629516), e(89.182663), e(69.626305), e(85.356495), e(66.955147), e(83.347236)),
  curve_to(e(66.955147), e(83.347236), e(66.949547), e(86.058459), e(72.815824), e(95.331646), e(67.818435), e(90.227994)),
  curve_to(e(67.818435), e(90.227994), e(64.717877), e(85.663331), e(68.928916), e(95.882084), e(64.398268), e(93.97607)),
  curve_to(e(64.398268), e(93.97607), e(62.712859), e(98.964732), e(61.768461), e(105.12732), e(55.208763), e(105.58497)),
  curve_to(e(55.208763), e(105.58497), e(51.602607), e(107.3734), e(47.789436), e(104.40418), e(45.464903), e(108.96019)),
  curve_to(e(45.464903), e(108.96019), e(42.265558), e(104.10894), e(31.890531), e(108.91119), e(31.9562), e(100.76264)),
  curve_to(e(31.9562), e(100.76264), e(28.697661), e(95.559482), e(24.843435), e(91.097005), e(20.462987), e(87.073272)),
  curve_to(e(20.462987), e(87.073272), e(18.222523), e(82.222649), e(14.739843), e(77.542793), e(14.593941), e(72.126685)),
  curve_to(e(14.593941), e(72.126685), e(12.63744), e(68.199932), e(16.964105), e(72.677953), e(15.00587), e(67.844708)),
  curve_to(e(15.00587), e(67.844708), e(14.112967), e(64.611583), e(11.888086), e(59.974351), e(15.132737), e(62.03455)),
  curve_to(e(15.132737), e(62.03455), e(18.571013), e(58.137014), e(20.208248), e(51.74604), e(22.328936), e(49.284637)),
  curve_to(e(22.328936), e(49.284637), e(25.360328), e(50.672634), e(25.283271), e(42.456568), e(25.732976), e(42.626965)),
  curve_to(e(25.732976), e(42.626965), e(25.327871), e(39.370133), e(37.123351), e(37.10301), e(34.394248), e(30.315283)),
  curve_to(e(34.394248), e(30.315283), e(37.027436), e(25.999652), e(35.203074), e(17.274845), e(42.064735), e(17.483854)),
  curve_to(e(42.064735), e(17.483854), e(41.816282), e(12.01766), e(44.538347), e(8.9574078), e(45.393854), e(3.9323162)),
  move_to(e(45.393854), e(3.9323162), e(89.283597), e(4.702312)),
  curve_to(e(89.283597), e(4.702312), e(93.392862), e(9.8834215), e(80.76606), e(6.4047269), e(87.190521), e(5.2559712)),
  curve_to(e(87.190521), e(5.2559712), e(87.984422), e(6.5912779), e(88.619691), e(5.0547754), e(89.283597), e(4.702312)),
  move_to(e(89.283597), e(4.702312), e(66.491732), e(4.702312)),
  curve_to(e(66.491732), e(4.702312), e(69.923218), e(8.1817422), e(68.757693), e(8.7882078), e(66.491732), e(4.702312)),
  move_to(e(66.491732), e(4.702312), e(69.109717), e(7.0122993)),
  curve_to(e(69.109717), e(7.0122993), e(68.91906), e(7.4433736), e(69.540792), e(6.8216446), e(69.109717), e(7.0122993)),
  move_to(e(69.109717), e(7.0122993), e(66.953727), e(7.6282956)),
  curve_to(e(66.953727), e(7.6282956), e(67.144386), e(8.0593739), e(66.522652), e(7.4376417), e(66.953727), e(7.6282956)),
  move_to(e(66.953727), e(7.6282956), e(69.417713), e(7.7822943)),
  curve_to(e(69.417713), e(7.7822943), e(69.608369), e(8.2133702), e(68.986638), e(7.5916395), e(69.417713), e(7.7822943)),
  move_to(e(69.417713), e(7.7822943), e(81.891643), e(8.5522908)),
  curve_to(e(81.891643), e(8.5522908), e(81.700987), e(8.9833644), e(82.322715), e(8.3616369), e(81.891643), e(8.5522908)),
  move_to(e(81.891643), e(8.5522908), e(69.109717), e(13.172263)),
  curve_to(e(69.109717), e(13.172263), e(69.765044), e(14.501613), e(67.477591), e(13.308673), e(69.109717), e(13.172263)),
  move_to(e(69.109717), e(13.172263), e(68.647719), e(13.942259)),
  curve_to(e(68.647719), e(13.942259), e(68.838374), e(14.373338), e(68.216639), e(13.751599), e(68.647719), e(13.942259)),
  move_to(e(68.647719), e(13.942259), e(69.109717), e(15.020254)),
  curve_to(e(69.109717), e(15.020254), e(64.953721), e(19.867506), e(65.841586), e(12.325468), e(69.109717), e(15.020254)),
  move_to(e(69.109717), e(15.020254), e(68.185722), e(14.404257)),
  curve_to(e(68.185722), e(14.404257), e(68.376375), e(14.835334), e(67.754642), e(14.213595), e(68.185722), e(14.404257)),
  move_to(e(68.185722), e(14.404257), e(108.07148), e(16.560244)),
  line_to(e(108.07148), e(16.560244), e(108.07148), e(17.071266)),
  line_to(e(108.07148), e(17.071266), e(108.07148), e(16.560244)),
  move_to(e(108.07148), e(16.560244), e(40.157886), e(17.02224)),
  curve_to(e(40.157886), e(17.02224), e(40.348543), e(17.45332), e(39.726805), e(16.831582), e(40.157886), e(17.02224)),
  move_to(e(40.157886), e(17.02224), e(96.367556), e(17.638237)),
  curve_to(e(96.367556), e(17.638237), e(96.558215), e(18.069316), e(95.93648), e(17.447582), e(96.367556), e(17.638237)),
  move_to(e(96.367556), e(17.638237), e(108.84149), e(18.254235)),
  curve_to(e(108.84149), e(18.254235), e(110.03226), e(18.883016), e(107.65709), e(18.951048), e(108.84149), e(18.254235)),
  move_to(e(108.84149), e(18.254235), e(56.48179), e(24.876193)),
  curve_to(e(56.48179), e(24.876193), e(56.672448), e(25.307274), e(56.050709), e(24.685537), e(56.48179), e(24.876193)),
  move_to(e(56.48179), e(24.876193), e(58.021779), e(27.34018)),
  curve_to(e(58.021779), e(27.34018), e(55.954037), e(27.34898), e(57.855903), e(25.971517), e(58.021779), e(27.34018)),
  move_to(e(58.021779), e(27.34018), e(33.535924), e(34.116141)),
  curve_to(e(33.535924), e(34.116141), e(33.663575), e(35.73614), e(32.334126), e(35.243504), e(33.535924), e(34.116141)),
  move_to(e(33.535924), e(34.116141), e(139.6413), e(39.506109)),
  line_to(e(139.6413), e(39.506109), e(139.79017), e(40.250434)),
  line_to(e(139.79017), e(40.250434), e(139.6413), e(39.506109)),
  move_to(e(139.6413), e(39.506109), e(135.63733), e(40.276105)),
  curve_to(e(135.63733), e(40.276105), e(135.44667), e(40.707184), e(136.06842), e(40.085448), e(135.63733), e(40.276105)),
  move_to(e(135.63733), e(40.276105), e(139.9493), e(40.584103)),
  curve_to(e(139.9493), e(40.584103), e(141.33001), e(42.378646), e(139.25681), e(42.257122), e(139.9493), e(40.584103)),
  move_to(e(139.9493), e(40.584103), e(137.02333), e(40.892101)),
  curve_to(e(137.02333), e(40.892101), e(137.21397), e(41.323181), e(136.59224), e(40.70144), e(137.02333), e(40.892101)),
  move_to(e(137.02333), e(40.892101), e(139.17931), e(42.58609)),
  curve_to(e(139.17931), e(42.58609), e(139.36997), e(43.017169), e(138.74823), e(42.395437), e(139.17931), e(42.58609)),
  move_to(e(139.17931), e(42.58609), e(25.373963), e(42.894091)),
  curve_to(e(25.373963), e(42.894091), e(25.941157), e(43.683407), e(23.708748), e(42.870391), e(25.373963), e(42.894091)),
  move_to(e(25.373963), e(42.894091), e(23.217986), e(46.898067)),
  curve_to(e(23.217986), e(46.898067), e(23.408641), e(47.329145), e(22.786904), e(46.707409), e(23.217986), e(46.898067)),
  move_to(e(23.217986), e(46.898067), e(21.831986), e(47.360063)),
  line_to(e(21.831986), e(47.360063), e(21.322275), e(47.487492)),
  line_to(e(21.322275), e(47.487492), e(21.831986), e(47.360063)),
  move_to(e(21.831986), e(47.360063), e(23.217986), e(48.438057)),
  curve_to(e(23.217986), e(48.438057), e(23.408641), e(48.869137), e(22.786904), e(48.2474), e(23.217986), e(48.438057)),
  move_to(e(23.217986), e(48.438057), e(20.292002), e(53.05803)),
  curve_to(e(20.292002), e(53.05803), e(20.482656), e(53.489107), e(19.860922), e(52.867373), e(20.292002), e(53.05803)),
  move_to(e(20.292002), e(53.05803), e(146.26327), e(56.44601)),
  curve_to(e(146.26327), e(56.44601), e(146.40118), e(57.937547), e(146.0256), e(58.962946), e(146.26327), e(56.44601)),
  move_to(e(146.26327), e(56.44601), e(143.33728), e(59.833991)),
  line_to(e(143.33728), e(59.833991), e(143.07818), e(60.222651)),
  line_to(e(143.07818), e(60.222651), e(143.33728), e(59.833991)),
  move_to(e(143.33728), e(59.833991), e(84.355628), e(79.699876)),
  curve_to(e(84.355628), e(79.699876), e(84.546286), e(80.130951), e(83.924548), e(79.509215), e(84.355628), e(79.699876)),
  move_to(e(84.355628), e(79.699876), e(80.043652), e(81.085865)),
  curve_to(e(80.043652), e(81.085865), e(80.234308), e(81.51695), e(79.612574), e(80.895203), e(80.043652), e(81.085865)),
  move_to(e(80.043652), e(81.085865), e(135.63733), e(81.855858)),
  line_to(e(135.63733), e(81.855858), e(135.12631), e(81.855858)),
  line_to(e(135.12631), e(81.855858), e(135.63733), e(81.855858)),
  move_to(e(135.63733), e(81.855858), e(77.733666), e(86.01384)),
  line_to(e(77.733666), e(86.01384), e(77.267372), e(85.547546)),
  line_to(e(77.267372), e(85.547546), e(77.733666), e(86.01384)),
  move_to(e(77.733666), e(86.01384), e(112.38347), e(86.783825)),
  curve_to(e(112.38347), e(86.783825), e(112.57413), e(87.214911), e(111.9524), e(86.593172), e(112.38347), e(86.783825)),
  move_to(e(112.38347), e(86.783825), e(110.38148), e(86.783825)),
  curve_to(e(110.38148), e(86.783825), e(110.57214), e(87.214911), e(109.9504), e(86.593172), e(110.38148), e(86.783825)),
  move_to(e(110.38148), e(86.783825), e(111.30547), e(87.091824)),
  curve_to(e(111.30547), e(87.091824), e(111.49614), e(87.522918), e(110.8744), e(86.901179), e(111.30547), e(87.091824)),
  move_to(e(111.30547), e(87.091824), e(72.959693), e(90.325809)),
  line_to(e(72.959693), e(90.325809), e(72.652657), e(90.939888)),
  line_to(e(72.652657), e(90.939888), e(72.959693), e(90.325809)),
  move_to(e(72.959693), e(90.325809), e(71.5737), e(94.637794)),
  curve_to(e(71.5737), e(94.637794), e(67.132557), e(95.554633), e(67.071274), e(92.159973), e(71.225288), e(93.143813)),
  curve_to(e(71.225288), e(93.143813), e(71.845929), e(93.307506), e(72.130609), e(94.204241), e(71.5737), e(94.637794)),
  move_to(e(71.5737), e(94.637794), e(48.473836), e(104.03174)),
  curve_to(e(48.473836), e(104.03174), e(47.707662), e(107.55791), e(51.527756), e(104.12954), e(48.473836), e(104.03174)),
  move_to(e(48.473836), e(104.03174), e(57.251785), e(104.64773)),
  curve_to(e(57.251785), e(104.64773), e(57.442442), e(105.07881), e(56.820708), e(104.45707), e(57.251785), e(104.64773)),
  move_to(e(57.251785), e(104.64773), e(52.785812), e(109.88371)),
  curve_to(e(52.785812), e(109.88371), e(54.225138), e(113.34177), e(50.542654), e(110.656), e(52.785812), e(109.88371)),
  move_to(e(52.785812), e(109.88371), e(44.777859), e(110.6537)),
  curve_to(e(44.777859), e(110.6537), e(44.968516), e(111.08478), e(44.34678), e(110.46304), e(44.777859), e(110.6537)),
  move_to(e(44.777859), e(110.6537), e(42.929868), e(110.8077)),
  curve_to(e(42.929868), e(110.8077), e(43.269929), e(111.32716), e(42.064636), e(111.26297), e(42.929868), e(110.8077)),
  move_to(e(42.929868), e(110.8077), e(41.081878), e(112.03968)),
  curve_to(e(41.081878), e(112.03968), e(41.272539), e(112.47076), e(40.650801), e(111.84903), e(41.081878), e(112.03968)),
  move_to(e(41.081878), e(112.03968), e(41.543876), e(112.19369)),
  curve_to(e(41.543876), e(112.19369), e(43.422793), e(117.34227), e(37.772931), e(116.02212), e(41.543876), e(112.19369)),
  move_to(e(41.543876), e(112.19369), e(50.629826), e(112.96368)),
  curve_to(e(50.629826), e(112.96368), e(53.063972), e(118.25581), e(49.878028), e(128.15038), e(44.507375), e(124.76277)),
  curve_to(e(44.507375), e(124.76277), e(42.664879), e(123.57506), e(38.515155), e(113.64058), e(46.205665), e(116.30485)),
  curve_to(e(46.205665), e(116.30485), e(47.969765), e(116.0851), e(50.624773), e(114.47147), e(50.629826), e(112.96368)),
  end()
};

PROGMEM static int32_t arduino_program[] = {
  move_to(e(0.0), e(0.0), e(157.10609), e(48.631198)),
  curve_to(e(157.10609), e(48.631198), e(157.10609), e(86.448217), e(105.11056), e(100.53028), e(79.423585), e(62.021249)),
  curve_to(e(79.423585), e(62.021249), e(52.802707), e(101.94644), e(2.8939181), e(85.209645), e(2.8939181), e(48.57149)),
  curve_to(e(2.8939181), e(48.57149), e(2.8939181), e(11.933334), e(52.566934), e(-5.6822527), e(79.423585), e(35.227369)),
  curve_to(e(79.423585), e(35.227369), e(104.87479), e(-5.2091763), e(157.10609), e(10.812649), e(157.10609), e(48.631198)),
  line_to(e(157.10609), e(48.631198), e(157.10609), e(48.631198)),
  line_to(e(157.10609), e(48.631198), e(157.10609), e(48.631198)),
  move_to(e(157.10609), e(48.631198), e(143.45271), e(48.631198)),
  curve_to(e(143.45271), e(48.631198), e(143.45271), e(21.341277), e(104.8855), e(12.727919), e(86.943815), e(48.631198)),
  curve_to(e(86.943815), e(48.631198), e(105.23916), e(83.200984), e(143.45271), e(75.92112), e(143.45271), e(48.631198)),
  line_to(e(143.45271), e(48.631198), e(143.45271), e(48.631198)),
  line_to(e(143.45271), e(48.631198), e(143.45271), e(48.631198)),
  move_to(e(143.45271), e(48.631198), e(71.904886), e(48.631198)),
  curve_to(e(71.904886), e(48.631198), e(55.740679), e(12.020601), e(16.126277), e(22.108303), e(16.247225), e(48.690907)),
  curve_to(e(16.247225), e(48.690907), e(16.362049), e(75.27198), e(56.213756), e(83.556174), e(71.904886), e(48.631198)),
  line_to(e(71.904886), e(48.631198), e(71.904886), e(48.631198)),
  move_to(e(71.904886), e(48.631198), e(128.95729), e(44.800655)),
  line_to(e(128.95729), e(44.800655), e(106.07203), e(44.800655)),
  line_to(e(106.07203), e(44.800655), e(106.07203), e(52.349974)),
  line_to(e(106.07203), e(52.349974), e(128.95729), e(52.349974)),
  line_to(e(128.95729), e(52.349974), e(128.95729), e(44.800655)),
  move_to(e(128.95729), e(44.800655), e(53.45644), e(52.23209)),
  line_to(e(53.45644), e(52.23209), e(53.45644), e(44.208164)),
  line_to(e(53.45644), e(44.208164), e(45.90559), e(44.208164)),
  line_to(e(45.90559), e(44.208164), e(45.90559), e(36.658845)),
  line_to(e(45.90559), e(36.658845), e(37.883196), e(36.658845)),
  line_to(e(37.883196), e(36.658845), e(37.883196), e(44.208164)),
  line_to(e(37.883196), e(44.208164), e(30.333877), e(44.208164)),
  line_to(e(30.333877), e(44.208164), e(30.333877), e(52.23209)),
  line_to(e(30.333877), e(52.23209), e(37.883196), e(52.23209)),
  line_to(e(37.883196), e(52.23209), e(37.883196), e(59.78294)),
  line_to(e(37.883196), e(59.78294), e(45.90559), e(59.78294)),
  line_to(e(45.90559), e(59.78294), e(45.90559), e(52.23209)),
  line_to(e(45.90559), e(52.23209), e(53.45644), e(52.23209)),
  move_to(e(53.45644), e(52.23209), e(91.761845), e(105.58867)),
  curve_to(e(91.761845), e(105.58867), e(91.725105), e(113.01092), e(95.538801), e(116.60415), e(102.91052), e(116.36991)),
  line_to(e(102.91052), e(116.36991), e(109.2228), e(116.36991)),
  line_to(e(109.2228), e(116.36991), e(109.2228), e(105.58867)),
  line_to(e(109.2228), e(105.58867), e(109.2228), e(94.819678)),
  line_to(e(109.2228), e(94.819678), e(102.91052), e(94.819678)),
  curve_to(e(102.91052), e(94.819678), e(95.538801), e(94.585436), e(91.725101), e(98.17408), e(91.761845), e(105.58867)),
  line_to(e(91.761845), e(105.58867), e(91.761845), e(105.58867)),
  line_to(e(91.761845), e(105.58867), e(91.761845), e(105.58867)),
  move_to(e(91.761845), e(105.58867), e(97.040703), e(105.58867)),
  curve_to(e(97.040703), e(105.58867), e(97.017743), e(101.34629), e(99.240738), e(99.326916), e(103.70664), e(99.5336)),
  line_to(e(103.70664), e(99.5336), e(103.70664), e(105.58867)),
  line_to(e(103.70664), e(105.58867), e(103.70664), e(111.65293)),
  curve_to(e(103.70664), e(111.65293), e(99.240738), e(111.85655), e(97.017738), e(109.83717), e(97.040703), e(105.58867)),
  line_to(e(97.040703), e(105.58867), e(97.040703), e(105.58867)),
  move_to(e(97.040703), e(105.58867), e(79.402151), e(116.36991)),
  curve_to(e(79.402151), e(116.36991), e(85.46947), e(116.36991), e(87.837914), e(114.35054), e(88.075218), e(109.23396)),
  line_to(e(88.075218), e(109.23396), e(88.075218), e(94.808961)),
  line_to(e(88.075218), e(94.808961), e(82.529967), e(94.808961)),
  line_to(e(82.529967), e(94.808961), e(82.529967), e(108.99819)),
  curve_to(e(82.529967), e(108.99819), e(82.650915), e(111.2763), e(81.606779), e(112.24236), e(79.402151), e(112.24236)),
  curve_to(e(79.402151), e(112.24236), e(77.199055), e(112.24236), e(76.168697), e(111.2763), e(76.289646), e(108.99819)),
  line_to(e(76.289646), e(108.99819), e(76.289646), e(94.808961)),
  line_to(e(76.289646), e(94.808961), e(70.768891), e(94.808961)),
  line_to(e(70.768891), e(94.808961), e(70.768891), e(109.23396)),
  curve_to(e(70.768891), e(109.23396), e(71.004663), e(114.35054), e(73.333301), e(116.36991), e(79.402151), e(116.36991)),
  line_to(e(79.402151), e(116.36991), e(79.402151), e(116.36991)),
  move_to(e(79.402151), e(116.36991), e(55.694749), e(99.421838)),
  line_to(e(55.694749), e(99.421838), e(50.330156), e(99.421838)),
  line_to(e(50.330156), e(99.421838), e(50.330156), e(94.808961)),
  line_to(e(50.330156), e(94.808961), e(58.404604), e(94.808961)),
  line_to(e(58.404604), e(94.808961), e(66.488239), e(94.808961)),
  line_to(e(66.488239), e(94.808961), e(66.488239), e(99.421838)),
  line_to(e(66.488239), e(99.421838), e(61.120583), e(99.421838)),
  line_to(e(61.120583), e(99.421838), e(61.120583), e(105.58867)),
  line_to(e(61.120583), e(105.58867), e(61.120583), e(111.75704)),
  line_to(e(61.120583), e(111.75704), e(66.488239), e(111.75704)),
  line_to(e(66.488239), e(111.75704), e(66.488239), e(116.36991)),
  line_to(e(66.488239), e(116.36991), e(58.404604), e(116.36991)),
  line_to(e(58.404604), e(116.36991), e(50.330156), e(116.36991)),
  line_to(e(50.330156), e(116.36991), e(50.330156), e(111.75704)),
  line_to(e(50.330156), e(111.75704), e(55.694749), e(111.75704)),
  line_to(e(55.694749), e(111.75704), e(55.694749), e(105.58867)),
  line_to(e(55.694749), e(105.58867), e(55.694749), e(99.421838)),
  move_to(e(55.694749), e(99.421838), e(40.416986), e(94.808961)),
  line_to(e(40.416986), e(94.808961), e(46.312834), e(94.808961)),
  line_to(e(46.312834), e(94.808961), e(46.312834), e(116.36991)),
  line_to(e(46.312834), e(116.36991), e(41.121242), e(116.36991)),
  line_to(e(41.121242), e(116.36991), e(41.121242), e(103.41007)),
  line_to(e(41.121242), e(103.41007), e(39.884201), e(105.64838)),
  line_to(e(39.884201), e(105.64838), e(33.985291), e(116.42656)),
  line_to(e(33.985291), e(116.42656), e(28.086382), e(116.42656)),
  line_to(e(28.086382), e(116.42656), e(28.086382), e(94.867139)),
  line_to(e(28.086382), e(94.867139), e(33.279504), e(94.867139)),
  line_to(e(33.279504), e(94.867139), e(33.279504), e(107.82545)),
  line_to(e(33.279504), e(107.82545), e(34.518076), e(105.58867)),
  line_to(e(34.518076), e(105.58867), e(40.416986), e(94.808961)),
  move_to(e(40.416986), e(94.808961), e(16.349801), e(99.290173)),
  curve_to(e(16.349801), e(99.290173), e(19.448528), e(99.290173), e(19.710328), e(102.05208), e(19.710328), e(105.58867)),
  curve_to(e(19.710328), e(105.58867), e(19.710328), e(109.12526), e(19.448528), e(111.89023), e(16.349801), e(111.89023)),
  curve_to(e(16.349801), e(111.89023), e(13.251075), e(111.89023), e(13.024488), e(109.12526), e(13.024488), e(105.58867)),
  curve_to(e(13.024488), e(105.58867), e(13.024488), e(102.05208), e(13.251075), e(99.290173), e(16.349801), e(99.290173)),
  line_to(e(16.349801), e(99.290173), e(16.349801), e(99.290173)),
  line_to(e(16.349801), e(99.290173), e(16.349801), e(99.290173)),
  move_to(e(16.349801), e(99.290173), e(16.349801), e(116.36991)),
  curve_to(e(16.349801), e(116.36991), e(22.55491), e(116.36991), e(25.465324), e(112.94202), e(25.465324), e(105.58867)),
  curve_to(e(25.465324), e(105.58867), e(25.465324), e(98.23532), e(22.55491), e(94.815085), e(16.349801), e(94.815085)),
  curve_to(e(16.349801), e(94.815085), e(10.143162), e(94.815085), e(7.3368551), e(98.23532), e(7.3368551), e(105.58867)),
  curve_to(e(7.3368551), e(105.58867), e(7.3368551), e(112.94202), e(10.144693), e(116.36991), e(16.349801), e(116.36991)),
  line_to(e(16.349801), e(116.36991), e(16.349801), e(116.36991)),
  move_to(e(16.349801), e(116.36991), e(132.88734), e(116.36991)),
  line_to(e(132.88734), e(116.36991), e(140.1427), e(94.808961)),
  line_to(e(140.1427), e(94.808961), e(145.9222), e(94.808961)),
  line_to(e(145.9222), e(94.808961), e(153.21278), e(116.36991)),
  line_to(e(153.21278), e(116.36991), e(146.9847), e(116.36991)),
  line_to(e(146.9847), e(116.36991), e(145.83493), e(111.85502)),
  line_to(e(145.83493), e(111.85502), e(140.02482), e(111.85502)),
  line_to(e(140.02482), e(111.85502), e(138.66836), e(116.36991)),
  line_to(e(138.66836), e(116.36991), e(132.88734), e(116.36991)),
  line_to(e(132.88734), e(116.36991), e(132.88734), e(116.36991)),
  line_to(e(132.88734), e(116.36991), e(132.88734), e(116.36991)),
  move_to(e(132.88734), e(116.36991), e(141.1455), e(107.60958)),
  line_to(e(141.1455), e(107.60958), e(144.80304), e(107.60958)),
  line_to(e(144.80304), e(107.60958), e(143.00413), e(100.85638)),
  line_to(e(143.00413), e(100.85638), e(141.1455), e(107.60958)),
  line_to(e(141.1455), e(107.60958), e(141.1455), e(107.60958)),
  move_to(e(141.1455), e(107.60958), e(129.90496), e(94.808961)),
  line_to(e(129.90496), e(94.808961), e(129.90496), e(116.36991)),
  line_to(e(129.90496), e(116.36991), e(124.53731), e(116.36991)),
  line_to(e(124.53731), e(116.36991), e(124.53731), e(109.58762)),
  line_to(e(124.53731), e(109.58762), e(122.29594), e(109.58762)),
  line_to(e(122.29594), e(109.58762), e(118.40263), e(116.37144)),
  line_to(e(118.40263), e(116.37144), e(112.09035), e(116.37144)),
  line_to(e(112.09035), e(116.37144), e(117.01555), e(108.40723)),
  curve_to(e(117.01555), e(108.40723), e(111.32945), e(105.99745), e(111.17328), e(94.810492), e(121.4692), e(94.810492)),
  line_to(e(121.4692), e(94.810492), e(129.90496), e(94.810492)),
  line_to(e(129.90496), e(94.810492), e(129.90496), e(94.808992)),
  line_to(e(129.90496), e(94.808992), e(129.90496), e(94.808961)),
  move_to(e(129.90496), e(94.808961), e(124.4776), e(105.42792)),
  line_to(e(124.4776), e(105.42792), e(124.4776), e(99.590247)),
  line_to(e(124.4776), e(99.590247), e(121.90553), e(99.590247)),
  curve_to(e(121.90553), e(99.590247), e(117.03698), e(99.642297), e(118.60166), e(105.51059), e(121.84583), e(105.42792)),
  line_to(e(121.84583), e(105.42792), e(124.4776), e(105.42792)),
  line_to(e(124.4776), e(105.42792), e(124.4776), e(105.42792)),
  end()
};

PROGMEM static int32_t hand_program[] = {
  move_to(e(0.0), e(0.0), e(129.4655), e(52.422114)),
  line_to(e(129.4655), e(52.422114), e(140.88369), e(52.422114)),
  curve_to(e(140.88369), e(52.422114), e(143.65173), e(59.18261), e(145.03576), e(65.597017), e(145.03576), e(71.665353)),
  curve_to(e(145.03576), e(71.665353), e(145.03576), e(77.946745), e(143.65173), e(84.015146), e(140.88369), e(89.870574)),
  line_to(e(140.88369), e(89.870574), e(132.10047), e(89.870574)),
  line_to(e(132.10047), e(89.870574), e(124.11572), e(96.817302)),
  curve_to(e(124.11572), e(96.817302), e(116.55686), e(100.86295), e(106.81548), e(103.68422), e(94.891554), e(105.28114)),
  line_to(e(94.891554), e(105.28114), e(85.309859), e(104.40281)),
  line_to(e(85.309859), e(104.40281), e(58.800506), e(119.65368)),
  curve_to(e(58.800506), e(119.65368), e(54.808214), e(121.94264), e(51.614323), e(123.08713), e(49.21881), e(123.08712)),
  curve_to(e(49.21881), e(123.08712), e(47.089631), e(123.08713), e(45.306379), e(122.39512), e(43.869033), e(121.01107)),
  curve_to(e(43.869033), e(121.01107), e(42.378639), e(119.57385), e(41.633404), e(117.84381), e(41.633297), e(115.821)),
  curve_to(e(41.633297), e(115.821), e(41.633404), e(112.84005), e(44.082051), e(109.85908), e(48.979258), e(106.87808)),
  line_to(e(48.979258), e(106.87808), e(71.735789), e(93.064472)),
  line_to(e(71.735789), e(93.064472), e(39.158031), e(101.60815)),
  curve_to(e(39.158031), e(101.60815), e(35.112529), e(102.67281), e(32.31787), e(103.20514), e(30.774042), e(103.20509)),
  curve_to(e(30.774042), e(103.20509), e(27.793189), e(103.20514), e(25.584074), e(102.56635), e(24.146706), e(101.28876)),
  curve_to(e(24.146706), e(101.28876), e(22.656334), e(99.958011), e(21.911089), e(97.988438), e(21.91097), e(95.380048)),
  curve_to(e(21.91097), e(95.380048), e(21.911089), e(90.908636), e(25.317923), e(87.688125), e(32.131441), e(85.718505)),
  line_to(e(32.131441), e(85.718505), e(65.587536), e(76.136811)),
  line_to(e(65.587536), e(76.136811), e(27.420449), e(76.136811)),
  curve_to(e(27.420449), e(76.136811), e(23.268497), e(76.136871), e(20.154456), e(75.391629), e(18.078294), e(73.901081)),
  curve_to(e(18.078294), e(73.901081), e(16.002387), e(72.410659), e(14.96437), e(70.174933), e(14.964242), e(67.193895)),
  curve_to(e(14.964242), e(67.193895), e(14.96437), e(61.285259), e(19.648753), e(58.330905), e(29.017389), e(58.330827)),
  line_to(e(29.017389), e(58.330827), e(67.184484), e(59.209148)),
  line_to(e(67.184484), e(59.209148), e(37.481228), e(49.228216)),
  curve_to(e(37.481228), e(49.228216), e(28.751355), e(46.300567), e(24.386355), e(42.707433), e(24.386237), e(38.448811)),
  curve_to(e(24.386237), e(38.448811), e(24.386355), e(36.000261), e(24.99853), e(34.163768), e(26.222742), e(32.939333)),
  curve_to(e(26.222742), e(32.939333), e(27.447176), e(31.715122), e(29.31029), e(31.102958), e(31.812059), e(31.102851)),
  curve_to(e(31.812059), e(31.102851), e(34.367284), e(31.102958), e(37.374864), e(31.528811), e(40.834823), e(32.38041)),
  line_to(e(40.834823), e(32.38041), e(79.321298), e(42.281488)),
  curve_to(e(79.321298), e(42.281488), e(81.131237), e(41.749271), e(83.925895), e(41.296797), e(87.705282), e(40.924078)),
  curve_to(e(87.705282), e(40.924078), e(86.747169), e(39.220763), e(85.735769), e(36.851967), e(84.671079), e(33.81767)),
  curve_to(e(84.671079), e(33.81767), e(80.998158), e(31.635271), e(78.25673), e(29.213235), e(76.44679), e(26.551539)),
  curve_to(e(76.44679), e(26.551539), e(74.58375), e(23.890078), e(73.652198), e(20.962338), e(73.652129), e(17.768328)),
  curve_to(e(73.652129), e(17.768328), e(73.652198), e(14.681015), e(75.541919), e(10.928189), e(79.321298), e(6.50983)),
  line_to(e(79.321298), e(6.50983), e(80.598858), e(4.9128788)),
  line_to(e(80.598858), e(4.9128788), e(96.967587), e(22.159928)),
  line_to(e(96.967587), e(22.159928), e(116.61006), e(36.85186)),
  curve_to(e(116.61006), e(36.85186), e(122.03971), e(42.654201), e(126.32486), e(47.844282), e(129.4655), e(52.422114)),
  move_to(e(129.4655), e(52.422114), e(127.54917), e(56.254793)),
  curve_to(e(127.54917), e(56.254793), e(121.95986), e(49.06861), e(116.98272), e(43.266364), e(112.6177), e(38.848055)),
  line_to(e(112.6177), e(38.848055), e(94.652011), e(25.433681)),
  line_to(e(94.652011), e(25.433681), e(80.838401), e(10.661899)),
  curve_to(e(80.838401), e(10.661899), e(78.7092), e(13.217145), e(77.644567), e(15.639181), e(77.644503), e(17.928019)),
  curve_to(e(77.644503), e(17.928019), e(77.644566), e(20.483255), e(78.496273), e(22.85205), e(80.199622), e(25.034438)),
  curve_to(e(80.199622), e(25.034438), e(81.849862), e(27.217051), e(84.378364), e(29.213235), e(87.785131), e(31.023)),
  curve_to(e(87.785131), e(31.023), e(90.712922), e(40.391861), e(96.036082), e(48.270135), e(103.75462), e(54.657843)),
  line_to(e(103.75462), e(54.657843), e(101.3592), e(57.692047)),
  curve_to(e(101.3592), e(57.692047), e(96.568398), e(54.498231), e(92.682491), e(50.13324), e(89.70147), e(44.597063)),
  curve_to(e(89.70147), e(44.597063), e(85.922079), e(45.07624), e(82.462026), e(45.768251), e(79.321298), e(46.673097)),
  line_to(e(79.321298), e(46.673097), e(41.713149), e(36.612319)),
  curve_to(e(41.713149), e(36.612319), e(37.880569), e(35.547788), e(35.085909), e(35.015474), e(33.32916), e(35.015378)),
  curve_to(e(33.32916), e(35.015378), e(31.732326), e(35.015474), e(30.561228), e(35.308257), e(29.815865), e(35.893694)),
  curve_to(e(29.815865), e(35.893694), e(29.017507), e(36.479344), e(28.618275), e(37.35766), e(28.618158), e(38.528651)),
  curve_to(e(28.618158), e(38.528651), e(28.618275), e(40.764484), e(31.359704), e(42.813894), e(36.842455), e(44.67691)),
  line_to(e(36.842455), e(44.67691), e(71.176857), e(56.254793)),
  line_to(e(71.176857), e(56.254793), e(71.176857), e(63.121675)),
  line_to(e(71.176857), e(63.121675), e(31.97175), e(62.163504)),
  line_to(e(31.97175), e(62.163504), e(29.416632), e(62.083654)),
  curve_to(e(29.416632), e(62.083654), e(22.443412), e(62.083728), e(18.956738), e(63.760523), e(18.956609), e(67.114044)),
  curve_to(e(18.956609), e(67.114044), e(18.956738), e(68.764292), e(19.568902), e(70.015235), e(20.793113), e(70.866874)),
  curve_to(e(20.793113), e(70.866874), e(21.96433), e(71.718646), e(23.774201), e(72.144498), e(26.222742), e(72.144434)),
  line_to(e(26.222742), e(72.144434), e(31.253126), e(72.064584)),
  line_to(e(31.253126), e(72.064584), e(71.176857), e(72.064584)),
  line_to(e(71.176857), e(72.064584), e(72.294722), e(78.372534)),
  line_to(e(72.294722), e(78.372534), e(35.644736), e(88.353466)),
  curve_to(e(35.644736), e(88.353466), e(29.097368), e(90.110157), e(25.823615), e(92.399115), e(25.823497), e(95.220349)),
  curve_to(e(25.823497), e(95.220349), e(25.823615), e(96.604411), e(26.249468), e(97.642427), e(27.101056), e(98.334399)),
  curve_to(e(27.101056), e(98.334399), e(27.899649), e(99.026449), e(29.150598), e(99.372452), e(30.853893), e(99.372414)),
  curve_to(e(30.853893), e(99.372414), e(31.971857), e(99.372452), e(34.234203), e(98.89337), e(37.64093), e(97.935162)),
  line_to(e(37.64093), e(97.935162), e(76.526636), e(87.2356)),
  line_to(e(76.526636), e(87.2356), e(79.321298), e(93.064466)),
  line_to(e(79.321298), e(93.064466), e(51.135143), e(110.39136)),
  curve_to(e(51.135143), e(110.39136), e(47.515495), e(112.57388), e(45.705612), e(114.46361), e(45.705515), e(116.06053)),
  curve_to(e(45.705515), e(116.06053), e(45.705612), e(117.12518), e(45.998394), e(117.92365), e(46.583842), e(118.45596)),
  curve_to(e(46.583842), e(118.45596), e(47.169482), e(118.98829), e(48.074418), e(119.25444), e(49.298661), e(119.25442)),
  curve_to(e(49.298661), e(119.25442), e(50.842467), e(119.25444), e(53.317734), e(118.26967), e(56.724462), e(116.30007)),
  line_to(e(56.724462), e(116.30007), e(84.112146), e(100.33058)),
  line_to(e(84.112146), e(100.33058), e(94.891554), e(101.28875)),
  curve_to(e(94.891554), e(101.28875), e(105.85731), e(99.638607), e(115.06637), e(96.897184), e(122.51877), e(93.064466)),
  line_to(e(122.51877), e(93.064466), e(131.30199), e(85.718499)),
  line_to(e(131.30199), e(85.718499), e(138.24872), e(85.718499)),
  curve_to(e(138.24872), e(85.718499), e(140.05861), e(81.03417), e(140.96354), e(76.083633), e(140.96354), e(70.866872)),
  curve_to(e(140.96354), e(70.866872), e(140.96354), e(66.555178), e(140.05861), e(61.684489), e(138.24872), e(56.254786)),
  line_to(e(138.24872), e(56.254786), e(127.54917), e(56.254786)),
  end()
};

static inline class fixed pgm_read_fixed(int32_t *mem) {
  return fixed::from_raw(pgm_read_dword(mem));
}

void Drawing::draw(int32_t *program, Display &display) {
  fixed x = 0;
  fixed y = 0;
  while (true) {
    switch (pgm_read_dword(program++)) {
      case oAbsMoveTo: {
        x = pgm_read_fixed(program++);
        y = pgm_read_fixed(program++);
        break;
      }
      case oAbsLineTo: {
        fixed tx = pgm_read_fixed(program++);
        fixed ty = pgm_read_fixed(program++);
        display.transform_line(Point<fixed>(x, y), Point<fixed>(tx, ty));
        x = tx;
        y = ty;
        break;
      }
      case oAbsCurveTo: {
        Point<fixed> points[] = {
#ifdef PRECOMPUTED
          Point<fixed>(pgm_read_fixed(program++), pgm_read_fixed(program++)),
          Point<fixed>(pgm_read_fixed(program++), pgm_read_fixed(program++)),
          Point<fixed>(pgm_read_fixed(program++), pgm_read_fixed(program++)),
          Point<fixed>(pgm_read_fixed(program++), pgm_read_fixed(program++))
        };
        x = pgm_read_fixed(program++);
        y = pgm_read_fixed(program++);
#else
          Point<fixed>(x, y),
          Point<fixed>(pgm_read_fixed(program++), pgm_read_fixed(program++)),
          Point<fixed>(pgm_read_fixed(program++), pgm_read_fixed(program++)),
          Point<fixed>(pgm_read_fixed(program++), pgm_read_fixed(program++))
        };
        x = points[3].x;
        y = points[3].y;
#endif
        display.transform_cubic_bezier(points);
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

void Drawing::draw_australia(Display &display) {
  draw(australia_program, display);
}

void Drawing::draw_arduino(Display &display) {
  draw(arduino_program, display);
}

