#include "set_BC_line.hpp"

#include "Core/MPMbox.hpp"

void set_BC_line::read(std::istream& is) { is >> line_num >> column0 >> column1 >> Xfixed >> Yfixed; }

void set_BC_line::exec() {
  if (box->nodes.empty()) {
    std::cerr << "@set_BC_line::exec, Cannot set BC. Grid not yet defined!" << std::endl;
  }

  node* N;
  size_t f = line_num * (box->Grid.Nx + 1);
  for (size_t i = column0; i <= column1; i++) {
    N = &(box->nodes[f + i]);
    N->xfixed = Xfixed;
    N->yfixed = Yfixed;
  }
}
