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

  // Number of GHOST node rings built around the element grid.
  //
  // A 16-node element needs the ring of nodes that surrounds it: for the element
  // (i, j) it reads the nodes i-1 .. i+2 and j-1 .. j+2. On the first and last
  // rows and columns of elements, that ring falls outside a grid that has only
  // Nx+1 by Ny+1 nodes. One ghost ring makes it exist everywhere, which is why
  // pad is 1 for the B-splines and 0 for the linear interpolations.
  //
  // The ghost nodes carry NEGATIVE logical indices, from -pad to Nx+pad. The
  // elements are unchanged -- there are still Nx by Ny of them, numbered
  // e = i + j * Nx, spanning the same domain [0, Nx*lx] x [0, Ny*ly] -- so
  // nothing outside the node numbering has to know about them.
  size_t pad{0};

  size_t nbNodeCols() const { return Nx + 1 + 2 * pad; }
  size_t nbNodeRows() const { return Ny + 1 + 2 * pad; }
  size_t nbNodes() const { return nbNodeCols() * nbNodeRows(); }

  // Number of the node of logical indices (i, j), which run from -pad to Nx+pad
  // (resp. Ny+pad). Everything that addresses a node by its position on the grid
  // has to go through here.
  size_t nodeNumber(long i, long j) const {
    return (size_t)(i + (long)pad) + (size_t)(j + (long)pad) * nbNodeCols();
  }

  grid(); // Ctor
};
