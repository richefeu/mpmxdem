#ifndef REACTIVATE_CHCL_BONDS_HPP
#define REACTIVATE_CHCL_BONDS_HPP

#include "Scheduler.hpp"

struct ReactivateCHCLBonds : public Scheduler {

  void read(std::istream& is);
  void write(std::ostream& os);
  void check();

 private:
  double bondingDistance;
  double timeBondReactivation;
};

#endif /* end of include guard: REACTIVATE_CHCL_BONDS_HPP */
