#pragma once

#include "Scheduler.hpp"

struct ReactivateCHCLBonds : public Scheduler {

  void read(std::istream& is);
  void write(std::ostream& os);
  void check();

 private:
  double bondingDistance;
  double timeBondReactivation;
};
