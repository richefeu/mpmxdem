#pragma once

#include "Scheduler.hpp"

#include "vec2.hpp"

struct GravityRamp : public Scheduler {

  void read(std::istream& is);
  void write(std::ostream& os);
  void check();

 private:
  vec2r gravityFrom;
  vec2r gravityTo;
  double rampStart{0.0};
  double rampStop{0.0};
};
