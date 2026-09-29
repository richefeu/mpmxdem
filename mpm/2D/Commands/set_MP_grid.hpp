#pragma once

#include "Command.hpp"

struct set_MP_grid : public Command {
  void read(std::istream& is);
  void exec();

 private:
  int groupNb{0};
  std::string modelName;
  double rho{0.0};
  double x0{0.0};
  double y0{0.0};
  double x1{0.0};
  double y1{0.0};
  double size{0.0};
  //double etaDamping;
};
