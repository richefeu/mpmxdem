#ifndef SPHERE_HPP
#define SPHERE_HPP

#include "vec3.hpp"

/// A sphere shape that is part of clumped-particle
struct Sphere {

  vec3r localPos; ///< Actual (not reduced) position expressed in the frame of the parent clump
  double radius;  ///< The particle radius
 
  Sphere(); // Ctor
};

#endif /* end of include guard: SPHERE_HPP */
