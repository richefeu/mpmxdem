#ifndef PROCESSED_DATA_MP_HPP
#define PROCESSED_DATA_MP_HPP

/**
 * @file ProcessedDataMP.hpp
 * @brief Header file for the ProcessedDataMP class and related declarations.
 *
 * This file contains the declaration of the ProcessedDataMP class, which is responsible
 * for holding the processed data for a material point.
 *
 * The class relies on various components such as the position, velocity, strain,
 * stress, and phase state of the material point.
 *
 * The file also defines constants and includes necessary headers for the implementation
 * of ProcessedDataMP.
 *
 * @note This file requires C++17 standard.
 */

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
