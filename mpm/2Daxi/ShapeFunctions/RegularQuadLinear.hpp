#ifndef REGULARQUADLINEAR_HPP
#define REGULARQUADLINEAR_HPP

#include "ShapeFunction.hpp"

struct RegularQuadLinear : public ShapeFunction {
  std::string getRegistrationName();
  void computeInterpolationValues(MPMbox& MPM, size_t p);
  RegularQuadLinear();
};

#endif /* end of include guard: REGULARQUADLINEAR_HPP */
