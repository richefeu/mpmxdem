#pragma once

#include "Command.hpp"

struct set_uniform_pressure : public Command {
  void read(std::istream& is);
  void exec();

 private:
  double pressure;
};
