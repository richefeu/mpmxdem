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

  // The logical indices of the grid, and the ghost ring if there is one.
  //
  // A boundary condition that reaches the edge of the grid is CONTINUED into the
  // ghost layer: the fixed line is the physical edge of the domain, and a
  // B-spline reads one node beyond it. Leaving that node free would let the
  // material slip through the very wall it is held by. With pad = 0 -- the
  // linear interpolations -- the loop below is exactly the historical one.
  const long pad = (long)box->Grid.pad;
  long j0 = (long)line_num, j1 = (long)line_num;
  if (line_num == 0) { j0 = -pad; }
  if (line_num == box->Grid.Ny) { j1 = (long)box->Grid.Ny + pad; }
  long i0 = (long)column0, i1 = (long)column1;
  if (column0 == 0) { i0 = -pad; }
  if (column1 == box->Grid.Nx) { i1 = (long)box->Grid.Nx + pad; }

  for (long j = j0; j <= j1; j++) {
    for (long i = i0; i <= i1; i++) {
      node &N = box->nodes[box->Grid.nodeNumber(i, j)];
      N.xfixed = Xfixed;
      N.yfixed = Yfixed;
    }
  }
}
