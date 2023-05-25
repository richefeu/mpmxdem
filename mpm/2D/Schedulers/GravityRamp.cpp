#include "GravityRamp.hpp"

#include "Core/MPMbox.hpp"

void GravityRamp::read(std::istream& is) {
  is >> gravityFrom >> rampStart;
  is >> gravityTo >> rampStop;
}

void GravityRamp::write(std::ostream& os) {
  os << "GravityRamp " << gravityFrom << ' ' << rampStart << ' ' << gravityTo << ' ' << rampStop << '\n';
}

void GravityRamp::check() {

  if (box->t <= rampStart) {
    box->gravity = gravityFrom;
  } else if (box->t >= rampStop) {
    box->gravity = gravityTo;
  } else {
    box->gravity = (box->t - rampStart) / (rampStop - rampStart) * (gravityTo - gravityFrom);
  }
}