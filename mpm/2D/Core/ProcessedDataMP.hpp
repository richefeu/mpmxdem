#ifndef PROCESSED_DATA_MP_HPP
#define PROCESSED_DATA_MP_HPP

#include <vector>

#include "mat4.hpp"
#include "vec2.hpp"

struct ProcessedDataMP {
  vec2r pos;    // position
  vec2r vel;    // Smoothed velocity
  mat4 strain;  // strain
  mat4 stress;  // Smoothed total stress
  double rho; // density

  vec2r corner[4];  // Four corners according to F (positions expressed in the global frame)

  // Ctor
  ProcessedDataMP();
};

#endif /* end of include guard: PROCESSED_DATA_MP_HPP */
