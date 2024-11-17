#ifndef PERIODICCELL_HPP
#define PERIODICCELL_HPP

#include "mat9.hpp"

/// A Periodic Cell
struct PeriodicCell {
  mat9r h;  ///< Matrix that hold the cell geometry (each column is a vector that defines a side)
  mat9r vh; ///< Velocities of the collective DoF
  mat9r ah; ///< Acceleration of the collective DoF

  mat9r velGrad; ///< velocity gradient tensor
  mat9r strain;  ///< Strain tensor

  double mass; ///< Mass of the periodic cell (a somehow fictive data)

  PeriodicCell();
  PeriodicCell(double XX, double XY, double XZ, double YX, double YY, double YZ, double ZX, double ZY, double ZZ);
  void Define(double XX, double XY, double XZ, double YX, double YY, double YZ, double ZX, double ZY, double ZZ);
  void update(double dt);
};

#endif /* end of include guard: PERIODICCELL_HPP */
