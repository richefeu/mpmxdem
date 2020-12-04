#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>

#include "quat.hpp"
#include "vec3.hpp"

/// A particle (which is a solid sphere)
struct Particle {

  // =========================================
  // Only position, velocity and acceleration
  // are expressed in the 'virtual world',
  // i.e., with reduced coordinates
  vec3r pos; ///< Reduced position
  vec3r vel; ///< Reduced velocity
  vec3r acc; ///< Reduced acceleration
  // =========================================

  quat Q;     ///< Angular position
  vec3r vrot; ///< Angular velocity
  vec3r arot; ///< Angular acceleration

  double radius;  ///< The particle radius
  double inertia; ///< Inertia value of a sphere (same value in the diagonal)
  double mass;    ///< The particle mass

  vec3r force;  ///< Resultant force acting on the particle
  vec3r moment; ///< Resultant moment acting on the particle

  Particle(); // Ctor
};

#endif /* end of include guard: PARTICLE_HPP */
