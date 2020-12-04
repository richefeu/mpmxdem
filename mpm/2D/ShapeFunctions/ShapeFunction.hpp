#ifndef SHAPEFUNCTION_HPP
#define SHAPEFUNCTION_HPP

#include <cstdlib>
struct MPMbox;

struct ShapeFunction {
  virtual ~ShapeFunction();

  // This function compute N and gradN of the MaterialPoint MPM.MP[p]
  virtual void computeInterpolationValues(MPMbox& MPM, size_t p) = 0;
};

#endif /* end of include guard: SHAPEFUNCTION_HPP */
