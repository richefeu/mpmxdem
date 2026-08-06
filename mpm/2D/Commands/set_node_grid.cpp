#include <string>

#include "Core/MPMbox.hpp"
#include "set_node_grid.hpp"

void set_node_grid::read(std::istream& is) {

  //   +---+---+---+
  //   |   |   |   |
  //   +---+---+---+
  //   |   |   |   |  Here Nx = nbElemX = 3, Ny = nbElemY = 3
  //   +---+---+---+ ^
  //   |   |   |   | ly
  // 0 +---+---+---+ v
  //   0   <lx>
  //
  // usage:
  //       1. set_node_grid  Nx.Ny.lx.ly  Nx Ny lx ly
  //       2. set_node_grid  W.H.lx.ly    TotalWidth TotalHeight lx ly
  //       3. set_node_grid  W.H.Nx.Ny    TotalWidth TotalHeight Nx Ny
  std::string inputChoice;
  is >> inputChoice;
  if (inputChoice == "Nx.Ny.lx.ly") {
    is >> nbElemX >> nbElemY >> lx >> ly;
  } else if (inputChoice == "W.H.lx.ly") {
    double W, H;
    is >> W >> H >> lx >> ly;
    nbElemX = static_cast<size_t>(fabs(round(W / lx)));
    nbElemY = static_cast<size_t>(fabs(round(H / ly)));
  } else if (inputChoice == "W.H.Nx.Ny") {
    double W, H;
    is >> W >> H >> nbElemX >> nbElemY;
    lx = W / (double)nbElemX;
    ly = H / (double)nbElemY;
  } else {
    Logger::error("@set_node_grid::read(), inputChoice: '{}' is not known", inputChoice);
  }
}

void set_node_grid::exec() {
  if (box->shapeFunction == nullptr) {
    Logger::critical("@set_node_grid::exec(), ShapeFunction has to be set BEFORE set_node_grid");
    Logger::critical("  It decides whether an element holds 4 or 16 nodes");
    exit(EXIT_FAILURE);
  }
  if (nbElemX == 0 || nbElemY == 0 || lx <= 0.0 || ly <= 0.0) {
    Logger::critical("@set_node_grid::exec(), invalid grid: Nx = {}, Ny = {}, lx = {}, ly = {}", nbElemX, nbElemY, lx,
                     ly);
    exit(EXIT_FAILURE);
  }

  box->Grid.Nx = nbElemX;
  box->Grid.Ny = nbElemY;
  box->Grid.lx = lx;
  box->Grid.ly = ly;

  if (element::nbNodes != 4 && element::nbNodes != 16) {
    Logger::critical("@set_node_grid::exec(), element::nbNodes = {}! It can only be 4 or 16", element::nbNodes);
    exit(EXIT_FAILURE);
  }

  box->buildGrid();
}
