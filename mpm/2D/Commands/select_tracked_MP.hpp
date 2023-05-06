#ifndef SELECT_TRACKED_MP_HPP
#define SELECT_TRACKED_MP_HPP

#include "Command.hpp"

#include <string>
#include <vector>

#include "ElementSelector.hpp"

struct select_tracked_MP : public Command {
  select_tracked_MP();
  void read(std::istream& is);
  void exec();

 private:
  ElementSelector<MPMbox> MP_Selector;
};

#endif /* end of include guard: SELECT_TRACKED_MP_HPP */
