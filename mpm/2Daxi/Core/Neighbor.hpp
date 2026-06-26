#pragma once

#include <cstddef>

// The Neighbor structure is used to store the information
// associated with a neighboring material point.
//
struct Neighbor {
  size_t PointNumber{0}; // Material Point number

  double fn{0.0};
  double dn{0.0};
  double ft{0.0};
  double dt{0.0};
  double sigma_n{0.0};

  Neighbor(); // Ctor
};
