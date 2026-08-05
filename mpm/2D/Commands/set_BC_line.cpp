#include "set_BC_line.hpp"

#include "Core/MPMbox.hpp"

void set_BC_line::read(std::istream& is) { is >> line_num >> column0 >> column1 >> Xfixed >> Yfixed; }

void set_BC_line::exec() {
  if (box->nodes.empty()) {
    Logger::critical("@set_BC_line::exec, the grid is not defined yet");
    Logger::critical("  set_node_grid has to come before this command");
    exit(EXIT_FAILURE);
  }

  // There are Nx+1 columns and Ny+1 lines of nodes for Nx x Ny elements, so the
  // indices go from 0 to Nx (resp. Ny) INCLUDED. The numbers being unsigned, a
  // negative value in the input file shows up here as a very large one.
  if (line_num > box->Grid.Ny || column1 < column0 || column1 > box->Grid.Nx) {
    Logger::critical("@set_BC_line::exec, 'set_BC_line {} {} {} ...' is outside the grid", line_num, column0, column1);
    Logger::critical("  The line number goes from 0 to {} and the column numbers from 0 to {}", box->Grid.Ny,
                     box->Grid.Nx);
    exit(EXIT_FAILURE);
  }

  size_t f = line_num * (box->Grid.Nx + 1);
  for (size_t i = column0; i <= column1; i++) {
    node& N = box->nodes[f + i];
    N.xfixed = Xfixed;
    N.yfixed = Yfixed;
  }
}
