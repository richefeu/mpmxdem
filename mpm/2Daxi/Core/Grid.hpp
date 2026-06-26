#pragma once

//
// Geometric grid definition
//
// This file contains the definition of the
// Geometric grid used in the simulations.
//
//

#include <cstddef>

//   +---+---+---+
//   |   |   |   |
//   +---+---+---+
//   |   |   |   |  Here Nx = nbElemX = 3, Ny = nbElemY = 3
//   +---+---+---+ ^
//   |   |   |   | ly
// 0 +---+---+---+ v
//   0   <lx>
//

struct grid {
  double lx{1.0};
  double ly{1.0}; // Size of quad-elements (regular grid)
  size_t Nx{20};
  size_t Ny{20}; // Number of grid-elements (QUA4) along x and y directions

  grid(); // Ctor
};
