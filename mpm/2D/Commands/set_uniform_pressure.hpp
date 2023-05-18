#ifndef SET_UNIFORME_PRESSURE_HPP
#define SET_UNIFORME_PRESSURE_HPP

#include "Command.hpp"

struct set_uniform_pressure : public Command {
  void read(std::istream& is);
  void exec();

 private:
  double pressure;
};

#endif /* end of include guard: SET_UNIFORME_PRESSURE_HPP */
