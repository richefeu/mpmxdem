#ifndef NEIGHBOR_HPP
#define NEIGHBOR_HPP

/**
 * @file
 * @brief Definition of the Neighbor structure.
 *
 * The Neighbor structure is used to store the information
 * associated with a neighboring material point.
 *
 * @see MaterialPoint
 */

#include <cstddef>

struct Neighbor {
  size_t PointNumber;  // Material Point number

  double fn, dn;
  double ft, dt;
  double sigma_n;
  Neighbor();  // Ctor
};

#endif /* end of include guard: NEIGHBOR_HPP */
