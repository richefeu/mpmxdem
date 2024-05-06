#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>

#include "quat.hpp"
#include "vec3.hpp"

// Struct to represent a particle, modeled as a solid sphere
struct Particle {

  // =========================================
  // The position, velocity, and acceleration of the particle
  // are expressed in 'reduced coordinates', i.e., in the 'virtual world'
  // =========================================
  vec3r pos;  ///< The reduced position of the particle in the virtual world
  vec3r vel;  ///< The reduced velocity of the particle in the virtual world
  vec3r acc;  ///< The reduced acceleration of the particle in the virtual world
  // =========================================

  quat Q;      ///< The angular position of the particle, represented as a quaternion
  vec3r vrot;  ///< The angular velocity of the particle
  vec3r arot;  ///< The angular acceleration of the particle

  double radius{0.0};   ///< The radius of the particle
  double inertia{0.0};  ///< The inertia of the particle (same value in the diagonal for a sphere)
  double mass{0.0};     ///< The mass of the particle

  vec3r force;   ///< The resultant force acting on the particle
  vec3r moment;  ///< The resultant moment acting on the particle

  size_t typeId{0};  ///< The ID of the type of the particle (possible values is limited to 0 or 1)
  size_t clusterId{0};  ///< The ID of the cluster to which the particle belongs

  // Default constructor for Particle
  Particle();
};


#endif /* end of include guard: PARTICLE_HPP */
