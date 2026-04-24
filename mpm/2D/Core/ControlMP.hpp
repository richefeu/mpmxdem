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
  size_t PointNumber{0}; // Material Point number

  bool xcontrol{FOR_CONTROL};
  bool ycontrol{FOR_CONTROL};
  double xvalue{0.0};
  double yvalue{0.0};
  ControlMP(); // Ctor
};

#endif /* end of include guard: CONTROL_MP_HPP */
