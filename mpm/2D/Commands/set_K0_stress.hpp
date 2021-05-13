#ifndef SET_K0_STRESS_HPP
#define SET_K0_STRESS_HPP

#include "Command.hpp"

struct set_K0_stress : public Command {
  void read(std::istream& is);
  void exec();

 private:
  double nu, rho0;
};

#endif /* end of include guard: SET_K0_STRESS_HPP */
