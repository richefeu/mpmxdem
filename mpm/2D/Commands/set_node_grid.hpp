#ifndef SET_NODE_GRIG_HPP
#define SET_NODE_GRIG_HPP

#include "Command.hpp"

struct set_node_grid : public Command {
  void read(std::istream& is);
  void exec();

 private:
  
  int nbElemX, nbElemY;
  double lx, ly;
};

#endif /* end of include guard: SET_NODE_GRIG_HPP */
