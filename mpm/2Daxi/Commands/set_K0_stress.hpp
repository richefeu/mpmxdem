#pragma once

#include "Command.hpp"

struct set_K0_stress : public Command {
  void read(std::istream& is);
  void exec();

 private:
  double nu, rho0;
};
