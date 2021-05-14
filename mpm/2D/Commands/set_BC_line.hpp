#ifndef SET_BC_LINE_HPP
#define SET_BC_LINE_HPP

#include "Command.hpp"

struct set_BC_line : public Command {
  void read(std::istream& is);
  void exec();

 private:
  int line_num, column0, column1;
  bool Xfixed, Yfixed;
};

#endif /* end of include guard: SET_BC_LINE_HPP */
