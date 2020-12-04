#ifndef COLORTABLE_HPP
#define COLORTABLE_HPP

#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

const float inv255 = 1.0 / 255.0;

struct colorRGBA {
  int r, g, b, a;
  float rr, gg, bb, aa;
  void set(int R, int G, int B, int A) {
    r = R;
    g = G;
    b = B;
    a = A;
    rr = r * inv255;
    gg = g * inv255;
    bb = b * inv255;
    aa = a * inv255;
  }
};

class ColorTable {
private:
  bool big_endian;
  int size;
  float curvature, bias;
  int rotation;
  bool swap, invert;
  std::vector<unsigned int> table;
  float min, max;
  int tableID;
  float alpha, beta, alphapow;

  unsigned int PACK_COLOR(int R, int G, int B, int A) {
    if (big_endian)
      return ((unsigned int)((R) << 24 | (G) << 16 | (B) << 8 | (A)));
    else
      return ((unsigned int)((A) << 24 | (B) << 16 | (G) << 8 | (R)));
  }
  int UNPACK_RED(unsigned int X) {
    if (big_endian)
      return (((X) >> 24) & 0xff);
    else
      return ((X)&0xff);
  }
  int UNPACK_GREEN(unsigned int X) {
    if (big_endian)
      return (((X) >> 16) & 0xff);
    else
      return (((X) >> 8) & 0xff);
  }
  int UNPACK_BLUE(unsigned int X) {
    if (big_endian)
      return (((X) >> 8) & 0xff);
    else
      return (((X) >> 16) & 0xff);
  }
  int UNPACK_ALPHA(unsigned int X) {
    if (big_endian)
      return ((X)&0xff);
    else
      return (((X) >> 24) & 0xff);
  }

  double gray(double s) { return s < 0. ? 0. : (s < 1. ? s : 1.); }
  double hot_r(double s) { return s < 0. ? 0. : (s < 3. / 8. ? 8. / 3. * s : 1.); }
  double hot_g(double s) { return s < 3. / 8. ? 0. : (s < 6. / 8. ? 8. / 3. * (s - 3. / 8.) : 1.); }
  double hot_b(double s) { return s < 6. / 8. ? 0. : (s < 1. ? 8. / 2. * (s - 6. / 8.) : 1.); }
  double cubic(double a, double b, double c, double d, double x) { return a + b * x + c * x * x + d * x * x * x; }

  void HSV_to_RGB(double H, double S, double V, double *R, double *G, double *B) {
    if (S < 5.0e-6) {
      *R = *G = *B = V;
    } else {
      int i = (int)H;
      double f = H - (float)i;
      double p1 = V * (1.0 - S);
      double p2 = V * (1.0 - S * f);
      double p3 = V * (1.0 - S * (1.0 - f));
      switch (i) {
      case 0:
        *R = V;
        *G = p3;
        *B = p1;
        break;
      case 1:
        *R = p2;
        *G = V;
        *B = p1;
        break;
      case 2:
        *R = p1;
        *G = V;
        *B = p3;
        break;
      case 3:
        *R = p1;
        *G = p2;
        *B = V;
        break;
      case 4:
        *R = p3;
        *G = p1;
        *B = V;
        break;
      case 5:
        *R = V;
        *G = p1;
        *B = p2;
        break;
      }
    }
  }

  void RGB_to_HSV(double R, double G, double B, double *H, double *S, double *V) {
    double maxv = R > G ? R : G;
    if (B > maxv)
      maxv = B;
    *V = maxv;
    if (maxv > 0) {
      double minv = R < G ? R : G;
      if (B < minv)
        minv = B;
      *S = 1.0 - double(minv) / maxv;
      if (maxv > minv) {
        if (maxv == R) {
          *H = (G - B) / double(maxv - minv);
          if (*H < 0)
            *H += 6.0;
        } else if (maxv == G)
          *H = 2.0 + (B - R) / double(maxv - minv);
        else
          *H = 4.0 + (R - G) / double(maxv - minv);
      }
    }
  }

public:
  ColorTable();
  void Rebuild();

  void setTableID(int id) { tableID = id; }
  void setSwap(bool s) { swap = s; }
  void setInvert(bool i) { invert = i; }
  void setMinMax(float Min, float Max) {
    min = Min;
    max = Max;
  }
  void setSize(float Size) { size = Size; }
  void setBias(float Bias) { bias = Bias; }
  void setCurvature(float Curv) { curvature = Curv; }
  void setRotation(int Rot) { rotation = Rot; }

  int getSize() { return size; }
  float getMin() { return min; }
  float getMax() { return max; }
  void getRGB(float value, colorRGBA *col) {
    unsigned int i;

    float pos = (value - min) / (max - min);                // in ]0.0 1.0[
    i = (unsigned int)(floor(pos * (float)(size - 2))) + 1; // in [1 size-2]
    // cout << "value = " << value << endl;
    // cout << "i = " << i << endl;

    if (value <= min)
      i = 0;
    else if (value >= max)
      i = size - 1;

    col->r = UNPACK_RED(table[i]);
    col->g = UNPACK_GREEN(table[i]);
    col->b = UNPACK_BLUE(table[i]);
    col->a = UNPACK_ALPHA(table[i]);

    col->rr = col->r * inv255;
    col->gg = col->g * inv255;
    col->bb = col->b * inv255;
    col->aa = col->a * inv255;
  }

  void Print() {
    int i, r, g, b, a;

    for (i = 0; i < size; i++) {
      r = UNPACK_RED(table[i]);
      g = UNPACK_GREEN(table[i]);
      b = UNPACK_BLUE(table[i]);
      a = UNPACK_ALPHA(table[i]);
      cout << i << " [" << r << ", " << g << ", " << b << ", " << a << "]" << endl;
    }
  }
};

#endif /* end of include guard: COLORTABLE_HPP */
