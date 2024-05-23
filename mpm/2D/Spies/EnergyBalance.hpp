#ifndef ENERGY_BALANCE_HPP
#define ENERGY_BALANCE_HPP

#include "Spy.hpp"

#include <vector>

struct EnergyBalance : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  std::string filename;
  std::ofstream file;

  double Wn_tot, Wt_tot, Wint_tot, Wp_tot;
};

#endif /* end of include guard: ENERGY_BALANCE_HPP */
