#pragma once

#include "Command.hpp"

struct move_MP : public Command {
  void read(std::istream& is);
  void exec();

 private:
  int groupNb{0};
  double x0{0.0};
  double y0{0.0};
  double dx{0.0};
  double dy{0.0};
  double thetaDeg{0.0};
};
