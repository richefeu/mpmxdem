#ifndef FRICTIONAL_VISCO_ELASTIC_HPP
#define FRICTIONAL_VISCO_ELASTIC_HPP

#include "BoundaryForceLaw.hpp"

struct frictionalViscoElastic : public BoundaryForceLaw {
  virtual void computeForces(MPMbox & MPM, size_t o);
};

#endif /* end of include guard: FRICTIONAL_VISCO_ELASTIC_HPP */
