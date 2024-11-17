#ifndef GRID_HPP
#define GRID_HPP

/**
 * @file
 * @brief   Geometric grid definition
 *
 * @details   This file contains the definition of the
 *            Geometric grid used in the simulations.
 *
 */

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
  double lx, ly;  // Size of quad-elements (regular grid)
  size_t Nx, Ny;  // Number of grid-elements (QUA4) along x and y directions

  grid();  // Ctor
};

#endif /* end of include guard: GRID_HPP */
