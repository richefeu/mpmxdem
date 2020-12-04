#ifndef NEW_SET_GRID_HPP
#define NEW_SET_GRID_HPP

#include "Command.hpp"

struct new_set_grid : public Command {
  void read(std::istream& is);
  void exec();

 private:
  //int groupNb;
  double lengthX;
  double lengthY;
  double spacing;
};

#endif /* end of include guard: NEW_SET_GRID_HPP */
