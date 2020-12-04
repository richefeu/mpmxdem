#ifndef SET_BC_COLUMN_HPP_5101B5C4
#define SET_BC_COLUMN_HPP_5101B5C4

#include "Command.hpp"

struct set_BC_column : public Command {
  void read(std::istream& is);
  void exec();

 private:
  int column_num, line0, line1;
  bool Xfixed, Yfixed;
};

#endif /* end of include guard: SET_BC_COLUMN_HPP_5101B5C4 */
