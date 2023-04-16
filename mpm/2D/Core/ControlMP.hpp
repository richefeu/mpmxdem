#ifndef CONTROL_MP_HPP
#define CONTROL_MP_HPP

#include <cstddef>

#define VEL_CONTROL true
#define FOR_CONTROL false

struct ControlMP {
  size_t PointNumber;  // Material Point number

	bool xcontrol;
	bool ycontrol;
  double xvalue;
	double yvalue;
  ControlMP();  // Ctor
};

#endif /* end of include guard: CONTROL_MP_HPP */
