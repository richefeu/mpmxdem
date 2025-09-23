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
  h.xz = XZ;

  h.yx = YX;
  h.yy = YY;
  h.yz = YZ;

  h.zx = ZX;
  h.zy = ZY;
  h.zz = ZZ;

  strain.reset();
}

/// Updates the cell's velocity gradient and strain based on its velocity history
void PeriodicCell::update(double dt) {

  // Compute the inverse of the cell's stiffness matrix (h)
  mat9r hinv = h.get_inverse();

  // Calculate the cell's velocity gradient at the current time step using the velocity history (vh)
  velGrad = vh * hinv;

  // Update the cell's strain tensor based on its velocity gradient and a half-step of time
  strain += 0.5 * dt * (velGrad + velGrad.transposed());
}
