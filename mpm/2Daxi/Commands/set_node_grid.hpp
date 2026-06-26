#pragma once

#include "Command.hpp"

struct set_node_grid : public Command {
  void read(std::istream& is);
  void exec();

 private:
  
  size_t nbElemX, nbElemY;
  double lx, ly;
};
