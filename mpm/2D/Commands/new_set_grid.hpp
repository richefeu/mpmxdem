#pragma once

#include "Command.hpp"

struct new_set_grid : public Command {
  void read(std::istream& is);
  void exec();

 private:
  //int groupNb;
  double lengthX{0.0};
  double lengthY{0.0};
  double spacing{0.0};
};
