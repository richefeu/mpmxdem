#include "set_BC_column.hpp"

#include "Core/MPMbox.hpp"

void set_BC_column::read(std::istream& is) { is >> column_num >> line0 >> line1 >> Xfixed >> Yfixed; }

void set_BC_column::exec() {
  if (box->nodes.empty()) {
    std::cerr << "@set_BC_column::exec, Cannot set BC. Grid not yet defined!" << std::endl;
  }

  node* N;
  for (int j = line0; j <= line1; j++) {
    N = &(box->nodes[j * (box->Grid.Nx + 1) + column_num]);
    N->xfixed = Xfixed;
    N->yfixed = Yfixed;
  }
}
