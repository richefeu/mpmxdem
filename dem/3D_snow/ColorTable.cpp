#include "ColorTable.hpp"

#define ALL_BLACK 1
#define VIS5D 2
#define MATLAB_JET 3
#define SAMCET 4

ColorTable::ColorTable() {
  short int word = 0x0001;
  char *byte = (char *)&word;
  big_endian = (byte[0] ? false : true);

  rotation = 0.0;
  bias = 0.0;
  curvature = 0.0;
  min = 0.0;
  max = 1.0;
  size = 256;
  swap = false;
  tableID = 2;
  invert = false;
  alphapow = 0.0;
  alpha = 0.0;
  beta = 0.0;

  Rebuild();
}

void ColorTable::Rebuild() {
  double s, t, gamma;
  int r, g, b, a;

  if (!table.empty())
    table.clear();
  table.reserve(size);

  for (int i = 0; i < size; i++) {

    if (size > 1) {
      if (i + rotation < 0)
        s = (double)(i + rotation + size) / (double)(size - 1);
      else if (i + rotation > size - 1)
        s = (double)(i + rotation - size) / (double)(size - 1);
      else
        s = (double)(i + rotation) / (double)(size - 1);
    } else
      s = 0.;

    if (swap)
      s = 1.0 - s;

    switch (tableID) {
    case 0: // all black
      r = g = b = 0;
      break;
    case 1: // vis5d
      t = (curvature + 1.4) * (s - (1. + bias) / 2.);
      r = (int)(128.0 + 127.0 * atan(7.0 * t) / 1.57);
      g = (int)(128.0 + 127.0 * (2 * exp(-7 * t * t) - 1));
      b = (int)(128.0 + 127.0 * atan(-7.0 * t) / 1.57);
      break;
    case 2: { // matlab "jet"
      double ii = (double)(s - bias) * 128.;
      if (ii < 0)
        ii = 0;
      if (ii > 128)
        ii = 128;
      double rr = ii <= 46 ? 0. : ii >= 111 ? -0.03125 * (ii - 111) + 1. : ii >= 78 ? 1. : 0.03125 * (ii - 46);
      double gg = ii <= 14 || ii >= 111 ? 0. : ii >= 79 ? -0.03125 * (ii - 111) : ii <= 46 ? 0.03125 * (ii - 14) : 1.;
      double bb = ii >= 79 ? 0. : ii >= 47 ? -0.03125 * (ii - 79) : ii <= 14 ? 0.03125 * (ii - 14) + 1. : 1.;
      r = (int)(rr * 255.);
      g = (int)(gg * 255.);
      b = (int)(bb * 255.);
    } break;
    case 3: // lucie, samcef (?)
      if (s - bias <= 0.) {
        r = 0;
        g = 0;
        b = 255;
      } else if (s - bias <= 0.40) {
        r = 0;
        g = (int)((s - bias) * 637.5);
        b = (int)(255. - (s - bias) * 637.5);
      } else if (s - bias <= 0.60) {
        r = (int)(1275. * (s - bias - 0.4));
        g = 255;
        b = 0;
      } else if (s - bias <= 1.) {
        r = 255;
        g = (int)(255. - 637.5 * (s - bias - 0.6));
        b = 0;
      } else {
        r = 255;
        g = 0;
        b = 0;
      }
      break;
    case 4: // rainbow
      if (s - bias <= 0.) {
        r = 0;
        g = 0;
        b = 255;
      } else if (s - bias <= 0.25 + curvature) {
        curvature = (curvature == -0.25) ? -0.26 : curvature;
        r = 0;
        g = (int)((s - bias) * (255. / (0.25 + curvature)));
        b = 255;
      } else if (s - bias <= 0.50) {
        curvature = (curvature == 0.25) ? 0.26 : curvature;
        r = 0;
        g = 255;
        b = (int)(255. - (255. / (0.25 - curvature)) * (s - bias - 0.25 - curvature));
      } else if (s - bias <= 0.75 - curvature) {
        curvature = (curvature == 0.25) ? 0.26 : curvature;
        r = (int)((s - bias - 0.5) * (255. / (0.25 - curvature)));
        g = 255;
        b = 0;
      } else if (s - bias <= 1.) {
        curvature = (curvature == -0.25) ? -0.26 : curvature;
        r = 255;
        g = (int)(255. - (255. / (0.25 + curvature)) * (s - bias - 0.75 + curvature));
        b = 0;
      } else {
        r = 255;
        g = 0;
        b = 0;
      }
      break;
    case 5: // emc2000 (rainbow with black and white)
      if (s - bias <= 0.) {
        r = 0;
        g = 0;
        b = 0;
      } else if (s - bias <= 0.2) {
        r = (int)(57 * (1 - 100 * ((s - bias) - 0.1) * ((s - bias) - 0.1)));
        g = 0;
        b = (int)((s - bias) * (255. / 0.2));
      } else if (s - bias <= 0.3624) {
        r = 0;
        g = (int)((s - bias - 0.2) * (255. / 0.1624));
        b = 255;
      } else if (s - bias <= 0.50) {
        r = 0;
        g = 255;
        b = (int)(255. - (255. / 0.1376) * (s - bias - 0.3624));
      } else if (s - bias <= 0.6376) {
        r = (int)((s - bias - 0.5) * (255. / 0.1376));
        g = 255;
        b = 0;
      } else if (s - bias <= 0.8) {
        r = 255;
        g = (int)(255. - (255. / 0.1624) * (s - bias - 0.6376));
        b = 0;
      } else if (s - bias <= 1.0) {
        r = 255;
        g = (int)((255. / 0.2) * (s - bias - 0.8));
        b = (int)(-3187.66 * (s - bias) * (s - bias) + 7012.76 * (s - bias) - 3570.61);
      } else {
        r = 255;
        g = 255;
        b = 255;
      }
      break;
    case 6: // darkblue->red->yellow->white
      r = (int)(255. * cubic(-0.0506169, 2.81633, -1.87033, 0.0524573, s - bias));
      g = (int)(255. * cubic(0.0485868, -1.26109, 6.3074, -4.12498, s - bias));
      b = (int)(255. * cubic(0.364662, 1.50814, -7.36756, 6.51847, s - bias));
      break;
    case 7: // matlab "hot"
      r = (int)(255. * hot_r(s - bias));
      g = (int)(255. * hot_g(s - bias));
      b = (int)(255. * hot_b(s - bias));
      break;
    case 8: // matlab "pink"
      r = (int)(255. * sqrt((2. * gray(s - bias) + hot_r(s - bias)) / 3.));
      g = (int)(255. * sqrt((2. * gray(s - bias) + hot_g(s - bias)) / 3.));
      b = (int)(255. * sqrt((2. * gray(s - bias) + hot_b(s - bias)) / 3.));
      break;
    case 9: // grayscale
      if (s - bias <= 0.) {
        r = g = b = 0;
      } else if (s - bias <= 1.) {
        r = g = b = (int)(255 * (1. - curvature) * (s - bias));
      } else {
        r = g = b = (int)(255 * (1. - curvature));
      }
      break;
    case 10: // all white
      r = g = b = 255;
      break;
    case 11: { // matlab "hsv"
      double H = 6. * s + 1.e-10, R = 0.0, G = 0.0, B = 0.0;
      HSV_to_RGB(H, 1., 1., &R, &G, &B);
      r = (int)(255 * R);
      g = (int)(255 * G);
      b = (int)(255 * B);
    } break;
    case 12: { // spectrum (truncated hsv)
      double H = 5. * s + 1.e-10, R = 0.0, G = 0.0, B = 0.0;
      HSV_to_RGB(H, 1., 1., &R, &G, &B);
      r = (int)(255 * R);
      g = (int)(255 * G);
      b = (int)(255 * B);
    } break;
    case 13: // matlab "bone"
      r = (int)(255. * (7.0 * gray(s - bias) + hot_b(s - bias)) / 8.0);
      g = (int)(255. * (7.0 * gray(s - bias) + hot_g(s - bias)) / 8.0);
      b = (int)(255. * (7.0 * gray(s - bias) + hot_r(s - bias)) / 8.0);
      break;
    case 14: // matlab "spring"
      r = (int)(255. * 1.0);
      g = (int)(255. * gray(s - bias));
      b = (int)(255. * (1.0 - gray(s - bias)));
      break;
    case 15: // matlab "summer"
      r = (int)(255. * gray(s - bias));
      g = (int)(255. * (0.5 + gray(s - bias) / 2.0));
      b = (int)(255. * 0.4);
      break;
    case 16: // matlab "autumn"
      r = (int)(255. * 1.0);
      g = (int)(255. * gray(s - bias));
      b = (int)(255. * 0.0);
      break;
    case 17: // matlab "winter"
      r = (int)(255. * 0.0);
      g = (int)(255. * gray(s - bias));
      b = (int)(255. * (0.5 + (1.0 - gray(s - bias)) / 2.0));
      break;
    case 18: // matlab "cool"
      r = (int)(255. * gray(s - bias));
      g = (int)(255. * (1.0 - gray(s - bias)));
      b = (int)(255. * 1.0);
      break;
    case 19: // matlab "copper"
      r = (int)(255. * MIN(1., gray(s - bias) * 1.25));
      g = (int)(255. * MIN(1., gray(s - bias) * 0.7812));
      b = (int)(255. * MIN(1., gray(s - bias) * 0.4975));
      break;
    case 20: // Random colors
      // r = ran1(&seed1) * 255.;
      // g = ran1(&seed1) * 255.;
      // b = ran1(&seed1) * 255.;
      break;
    default:
      r = g = b = 0;
      break;
    }

    float aa = 1.0;
    if (alphapow)
      aa = pow(float(s ? s : 1.0e-10), float(alphapow));
    a = (int)(255.0 * aa * alpha);

    if (beta) {
      if (beta > 0.0)
        gamma = 1.0 - beta;
      else
        gamma = 1.0 / (1.001 + beta); // beta is thresholded to [-1,1]
      r = (int)(255.0 * pow((float)r / 255.0, gamma));
      g = (int)(255.0 * pow((float)g / 255.0, gamma));
      b = (int)(255.0 * pow((float)b / 255.0, gamma));
    }

    if (invert) {
      r = 255 - r;
      g = 255 - g;
      b = 255 - b;
    }

    // clamp to [0,255]
    r = r < 0 ? 0 : (r > 255 ? 255 : r);
    g = g < 0 ? 0 : (g > 255 ? 255 : g);
    b = b < 0 ? 0 : (b > 255 ? 255 : b);
    a = a < 0 ? 0 : (a > 255 ? 255 : a);

    table[i] = PACK_COLOR(r, g, b, a);
  }
}
