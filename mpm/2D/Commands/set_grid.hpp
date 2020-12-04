#ifndef SET_GRIG_HPP_7C1155E6
#define SET_GRIG_HPP_7C1155E6

#include "Command.hpp"

struct set_grid : public Command {
  void read(std::istream& is);
  void exec();

 private:
  int nbElemX, nbElemY;
  double lx, ly;
};

#endif /* end of include guard: SET_GRIG_HPP_7C1155E6 */
