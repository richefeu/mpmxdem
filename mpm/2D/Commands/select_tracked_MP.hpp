#ifndef SELECT_TRACKED_MP_HPP
#define SELECT_TRACKED_MP_HPP

#include "Command.hpp"

#include <string>
#include <vector>

struct select_tracked_MP : public Command {
  void read(std::istream& is);
  void exec();

 private:
  std::string inputOption;
  size_t MP_ID;
  std::vector<size_t> MP_ID_LIST;
};

#endif /* end of include guard: SELECT_TRACKED_MP_HPP */
