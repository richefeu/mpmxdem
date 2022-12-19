#ifndef NEIGHBOR_HPP
#define NEIGHBOR_HPP

#include <cstddef>

struct Neighbor {
  size_t PointNumber;  // Material Point number

  double fn, dn;
  double ft, dt;
  double sigma_n;
  Neighbor();  // Ctor
};

#endif /* end of include guard: NEIGHBOR_HPP */
