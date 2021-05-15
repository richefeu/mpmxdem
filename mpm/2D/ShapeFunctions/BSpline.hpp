#ifndef BSPLINE_HPP
#define BSPLINE_HPP

#include "ShapeFunction.hpp"

struct BSpline : public ShapeFunction {
  std::string getRegistrationName();
  void computeInterpolationValues(MPMbox& MPM, size_t p);
  BSpline();

 private:
   // VR: use static double TwoThirds instead !!!
  double TwoThirds;
  double FourThirds;
  double OneSixth;
};

#endif /* end of include guard: BSPLINE_HPP */
