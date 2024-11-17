#ifndef CONTROL_MP_HPP
#define CONTROL_MP_HPP

/**
 * @file
 * @brief A class for tracking and controlling Material Points.
 *
 * The functions and data structures here are used to control the movement
 * of material points. This includes velocity control and force control.
 */

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
