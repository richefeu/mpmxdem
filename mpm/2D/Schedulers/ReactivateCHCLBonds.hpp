#pragma once

#include "Scheduler.hpp"

struct ReactivateCHCLBonds : public Scheduler {

  void read(std::istream& is);
  void write(std::ostream& os);
  void check();

 private:
  double bondingDistance{0.0};
  double timeBondReactivation{0.0};
};
