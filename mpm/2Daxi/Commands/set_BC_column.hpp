#pragma once

#include "Command.hpp"

struct set_BC_column : public Command {
  void read(std::istream& is);

  void exec();

 private:
  int column_num, line0, line1;
  bool Xfixed, Yfixed;
};
