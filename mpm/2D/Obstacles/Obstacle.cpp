#include "Obstacle.hpp"
#include "factory.hpp"

bool Obstacle::inside(vec2r&) { return false; }

// Ctor
Obstacle::Obstacle() : isFree(false), pos(), vel(), acc(), force() {
  std::string defaultBoundary = "frictionalNormalRestitution";
  boundaryForceLaw = Factory<BoundaryForceLaw>::Instance()->Create(defaultBoundary);
}

// Dtor
Obstacle::~Obstacle() {}
