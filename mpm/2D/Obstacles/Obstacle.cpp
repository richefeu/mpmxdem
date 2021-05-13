#include "Obstacle.hpp"
#include "factory.hpp"

bool Obstacle::MPisInside(MaterialPoint&) { return false; }

// Ctor
Obstacle::Obstacle() : isFree(false), pos(), vel(), acc(), force() {
  std::string defaultBoundary = "frictionalNormalRestitution";
  boundaryForceLaw = Factory<BoundaryForceLaw>::Instance()->Create(defaultBoundary);
}

// Dtor
Obstacle::~Obstacle() {}
