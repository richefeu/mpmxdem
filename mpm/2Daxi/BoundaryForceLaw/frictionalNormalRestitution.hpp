#ifndef FRICTIONAL_NORMAL_RESTITUTION_HPP
#define FRICTIONAL_NORMAL_RESTITUTION_HPP

#include "BoundaryForceLaw.hpp"

struct frictionalNormalRestitution : public BoundaryForceLaw {
  virtual void computeForces(MPMbox & MPM, size_t o);
};

#endif /* end of include guard: FRICTIONAL_NORMAL_RESTITUTION_HPP */
