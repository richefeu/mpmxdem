#ifndef SET_BC_COLUMN_HPP
#define SET_BC_COLUMN_HPP

#include "Command.hpp"

struct set_BC_column : public Command {
  void read(std::istream& is);
  void exec();

 private:
  int column_num, line0, line1;
  bool Xfixed, Yfixed;
};

#endif /* end of include guard: SET_BC_COLUMN_HPP */
