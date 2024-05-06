#include "Particle.hpp"

Particle::Particle()
    : pos(),
      vel(),
      acc(),
      Q(),
      vrot(),
      arot(),
      radius(0.0),
      inertia(0.0),
      mass(0.0),
      force(),
      moment(),
      typeId(0),
      clusterId(0) {}
