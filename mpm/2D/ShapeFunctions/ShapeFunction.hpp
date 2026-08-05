#ifndef SHAPEFUNCTION_HPP
#define SHAPEFUNCTION_HPP

#include <cstdlib>
#include <string>
class MPMbox;

struct ShapeFunction {
  virtual ~ShapeFunction();
  
  virtual std::string getRegistrationName() = 0;

  // This function compute N and gradN of the MaterialPoint MPM.MP[p]
  virtual void computeInterpolationValues(MPMbox& MPM, size_t p) = 0;

 protected:
  // Set MPM.MP[p].e, the number of the element that holds the Material Point.
  // It has to be called at the very beginning of computeInterpolationValues,
  // BEFORE any use of MPM.Elem[MPM.MP[p].e]: this is the single place where
  // the index is checked against the actual extent of the grid.
  void locateElement(MPMbox& MPM, size_t p);
};

#endif /* end of include guard: SHAPEFUNCTION_HPP */
