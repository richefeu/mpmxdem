#ifndef GRID_HPP
#define GRID_HPP

struct grid {
  double lx, ly;  // Size of quad-elements (in case of regular grid)
  int Nx, Ny;     // Number of grid-elements (QUA4) along x and y directions

  grid();  // Ctor
};

#endif /* end of include guard: GRID_HPP */
