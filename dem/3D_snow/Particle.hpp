#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>

#include "Sphere.hpp"

#include "OBB.hpp"
#include "quat.hpp"
#include "vec3.hpp"

/// A particle (which is a clump of spheres)
struct Particle {

  // =========================================
  // Only position, velocity and acceleration
  // are expressed in the 'virtual world',
  // i.e., with reduced coordinates
  vec3r pos;  ///< Reduced position
  vec3r vel;  ///< Reduced velocity
  vec3r acc;  ///< Reduced acceleration
  // =========================================

  quat Q;      ///< Angular position
  vec3r vrot;  ///< Angular velocity
  vec3r arot;  ///< Angular acceleration

  vec3r I_m;    ///< Inertia values
  double mass;  ///< The particle mass
  double volume;

  vec3r force;   ///< Resultant force acting on the particle
  vec3r moment;  ///< Resultant moment acting on the particle

  std::vector<Sphere> subSpheres;
  OBB obb;

  Particle();  // Ctor

  void readShape(std::istream& is, double density);
	void writeShape(std::ostream& os);
  void fitObb();
  bool inside(const vec3r& point);
  void massProperties();
};

#endif /* end of include guard: PARTICLE_HPP */
