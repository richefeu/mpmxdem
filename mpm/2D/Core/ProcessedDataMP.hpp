#ifndef PROCESSED_DATA_MP_HPP
#define PROCESSED_DATA_MP_HPP

#include <vector>

#include "mat4.hpp"
#include "vec2.hpp"

struct ProcessedDataMP {
  vec2r pos;                // position
  vec2r vel;                // Smoothed velocity
  mat4r strain;             // strain
  mat4r stress;             // Smoothed total stress
  double outOfPlaneStress;  // third principal stress
  mat4r velGrad;            // Smothed velocity gradient
  double rho;               // density

  vec2r corner[4];  // Four corners according to F (positions expressed in the global frame)

  // Ctor
  ProcessedDataMP();
};

#endif /* end of include guard: PROCESSED_DATA_MP_HPP */
