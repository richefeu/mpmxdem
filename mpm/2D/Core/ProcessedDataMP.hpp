#ifndef PROCESSED_DATA_MP_HPP
#define PROCESSED_DATA_MP_HPP

#include <vector>

#include "mat4.hpp"
#include "vec2.hpp"

struct ProcessedDataMP {
  vec2r vel;    // Smoothed velocity
  mat4 stress;  // Smoothed total stress

  vec2r corner[4];  // Four corners according to F (positions expressed in the global frame)

  // Ctor
  ProcessedDataMP();
};
struct CutDataMP {
  vec2r pos;    // position
  vec2r vel;    // Smoothed velocity
  mat4 stress;  // Smoothed total stress
  mat4 strain;  // strain
  double rho; // density
  vec2r corner[4];  // Four corners according to F (positions expressed in the global frame)

  // Ctor
  CutDataMP();
};

#endif /* end of include guard: PROCESSED_DATA_MP_HPP */
