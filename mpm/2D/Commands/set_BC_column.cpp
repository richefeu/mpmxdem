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

  // See the note in set_BC_line.cpp: a boundary condition that reaches the edge
  // of the grid is continued into the ghost layer.
  const long pad = (long)box->Grid.pad;
  long i0 = (long)column_num, i1 = (long)column_num;
  if (column_num == 0) { i0 = -pad; }
  if ((size_t)column_num == box->Grid.Nx) { i1 = (long)box->Grid.Nx + pad; }
  long j0 = (long)line0, j1 = (long)line1;
  if (line0 == 0) { j0 = -pad; }
  if ((size_t)line1 == box->Grid.Ny) { j1 = (long)box->Grid.Ny + pad; }

  for (long j = j0; j <= j1; j++) {
    for (long i = i0; i <= i1; i++) {
      node &N = box->nodes[box->Grid.nodeNumber(i, j)];
      N.xfixed = Xfixed;
      N.yfixed = Yfixed;
    }
  }
}
