#ifndef SET_MP_GRID_HPP
#define SET_MP_GRID_HPP

#include "Command.hpp"

struct set_MP_grid : public Command {
  void read(std::istream& is);
  void exec();

 private:
  int groupNb;
  std::string modelName;
  double rho;
  double x0;
  double y0;
  double x1;
  double y1;
  double size;
};

#endif /* end of include guard: SET_MP_GRID_HPP */
