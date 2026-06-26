#ifndef FRICTIONAL_VISCO_ELASTOFRAGILE_HPP
#define FRICTIONAL_VISCO_ELASTOFRAGILE_HPP

#include "BoundaryForceLaw.hpp"

struct frictionalViscoElastofragile : public BoundaryForceLaw {
  virtual void computeForces(MPMbox & MPM, size_t o);
};

#endif /* end of include guard: FRICTIONAL_VISCO_ELASTIC_HPP */
