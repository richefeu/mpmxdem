#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>

#include "Sphere.hpp"

#include "quat.hpp"
#include "vec3.hpp"
#include "OBB.hpp"


/*
fichier shapes.txt :

numero nb_spheres
obb.center x y z
obb.e1 x y z
obb.e2 x y z
obb.e3 x y z
obb.extents lx ly lz
I_m1 I_m2 I_m3
nb fois [x y z r]
*/


/// A particle (which is a clump of spheres)
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

  //double radius;  ///< The particle radius -> remplacé par OBB
  vec3r inertia; ///< Inertia values 
	double mass;    ///< The particle mass
	
	// shape
	vec3r I_m;
	double volume;
  
  vec3r force;  ///< Resultant force acting on the particle
  vec3r moment; ///< Resultant moment acting on the particle
	
	std::vector<Sphere> subSpheres;
	OBB obb;

  Particle(); // Ctor
	
	void readShape(std::istream& is);
	void fitObb();
	void massProperties();
	
};

#endif /* end of include guard: PARTICLE_HPP */
