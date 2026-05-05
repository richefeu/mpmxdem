#pragma once

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
