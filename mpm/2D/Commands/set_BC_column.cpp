#include "set_BC_column.hpp"

#include "Core/MPMbox.hpp"

void set_BC_column::read(std::istream& is) { is >> column_num >> line0 >> line1 >> Xfixed >> Yfixed; }

void set_BC_column::exec() {
  if (box->nodes.empty()) {
    Logger::critical("@set_BC_column::exec, the grid is not defined yet");
    Logger::critical("  set_node_grid has to come before this command");
    exit(EXIT_FAILURE);
  }

  // There are Nx+1 columns and Ny+1 lines of nodes for Nx x Ny elements, so the
  // indices go from 0 to Nx (resp. Ny) INCLUDED.
  if (column_num < 0 || (size_t)column_num > box->Grid.Nx || line0 < 0 || line1 < line0 ||
      (size_t)line1 > box->Grid.Ny) {
    Logger::critical("@set_BC_column::exec, 'set_BC_column {} {} {} ...' is outside the grid", column_num, line0, line1);
    Logger::critical("  The column number goes from 0 to {} and the line numbers from 0 to {}", box->Grid.Nx,
                     box->Grid.Ny);
    exit(EXIT_FAILURE);
  }

  for (size_t j = (size_t)line0; j <= (size_t)line1; j++) {
    node& N = box->nodes[j * (box->Grid.Nx + 1) + (size_t)column_num];
    N.xfixed = Xfixed;
    N.yfixed = Yfixed;
  }
}
