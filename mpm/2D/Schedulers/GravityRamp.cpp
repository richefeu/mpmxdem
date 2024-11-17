#include "GravityRamp.hpp"

#include "Core/MPMbox.hpp"

void GravityRamp::read(std::istream& is) {
  is >> gravityFrom >> rampStart;
  is >> gravityTo >> rampStop;
}

void GravityRamp::write(std::ostream& os) {
  os << "GravityRamp " << gravityFrom << ' ' << rampStart << ' ' << gravityTo << ' ' << rampStop << '\n';
}

/**
 * @brief Ramp the gravity from @c gravityFrom to @c gravityTo between times @c rampStart and @c rampStop.
 *
 * This function is called by the MPMbox at each time-step. It updates the gravity of the MPMbox
 * according to the following rules:
 * - If the current time is less than or equal to @c rampStart, the gravity is set to @c gravityFrom.
 * - If the current time is greater than or equal to @c rampStop, the gravity is set to @c gravityTo.
 * - If the current time is between @c rampStart and @c rampStop, the gravity is set to a linear
 *   interpolation between @c gravityFrom and @c gravityTo.
 */
void GravityRamp::check() {

  if (box->t <= rampStart) {
    box->gravity = gravityFrom;
  } else if (box->t >= rampStop) {
    box->gravity = gravityTo;
  } else {
    box->gravity = (box->t - rampStart) / (rampStop - rampStart) * (gravityTo - gravityFrom);
  }
}