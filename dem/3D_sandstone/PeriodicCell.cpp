#include "PeriodicCell.hpp"

PeriodicCell::PeriodicCell() : h(), vh(), ah() {}

/// Defines each component of matrix h (cell geometry)
PeriodicCell::PeriodicCell(double XX, double XY, double XZ, double YX, double YY, double YZ, double ZX, double ZY,
                           double ZZ)
    : h(), vh(), ah() {
  Define(XX, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ);
}

/// Defines each component of matrix h (cell geometry)
/// Must be done at initiation
void PeriodicCell::Define(double XX, double XY, double XZ, double YX, double YY, double YZ, double ZX, double ZY,
                          double ZZ) {
  h.xx = XX;
  h.xy = XY;
  h.yz = XZ;

  h.yx = YX;
  h.yy = YY;
  h.yz = YZ;

  h.zx = ZX;
  h.zy = ZY;
  h.zz = ZZ;

  strain.reset();
}

void PeriodicCell::update(double dt) {
  mat9r hinv = h.get_inverse();
  velGrad = vh * hinv;
  strain += 0.5 * dt * (velGrad + velGrad.transposed());
}
