#ifndef LINEAR_HPP
#define LINEAR_HPP

#include "ShapeFunction.hpp"

struct Linear : public ShapeFunction {
  void computeInterpolationValues(MPMbox& MPM, size_t p);
  Linear();
};

#endif /* end of include guard: LINEAR_HPP */
