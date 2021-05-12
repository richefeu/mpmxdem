#ifndef BOUNDARY_FORCE_LAW_HPP
#define BOUNDARY_FORCE_LAW_HPP

#include "Core/Neighbor.hpp"
#include "DataTable.hpp"
#include "Obstacles/Obstacle.hpp"

#include "vec2.hpp"

#include <iostream>
#include <vector>

struct MPMbox;
struct MaterialPoint;
struct Obstacle;

struct BoundaryForceLaw {
  virtual void computeForces(MPMbox & MPM, size_t o) = 0;
  BoundaryForceLaw();
  virtual ~BoundaryForceLaw();
};

#endif /* end of include guard: BOUNDARY_FORCE_LAW_HPP */
