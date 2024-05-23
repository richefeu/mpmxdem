#include <string>

#include "Core/MPMbox.hpp"
#include "set_node_grid.hpp"

//#include "spdlog/sinks/stdout_color_sinks.h"
//#include "spdlog/spdlog.h"

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
    is >> W >> H >> nbElemX >> nbElemX;
    lx = W / (double)nbElemX;
    ly = H / (double)nbElemY;
  } else {
    Logger::error("@set_node_grid::read(), inputChoice: '{}' is not known", inputChoice);
  }
}

void set_node_grid::exec() {
  if (box->shapeFunction == nullptr) {
    Logger::critical("@set_node_grid::exec(), ShapeFunction has to be set BEFORE set_node_grid");
    exit(0);
  }

  box->Grid.Nx = nbElemX;
  box->Grid.Ny = nbElemY;
  box->Grid.lx = lx;
  box->Grid.ly = ly;

  // Create the nodes and set their positions
  if (!box->nodes.empty()) box->nodes.clear();
  node N;
  N.mass = 0.0;
  N.number = 0;
  N.q.reset();
  N.f.reset();
  N.fb.reset();
  N.xfixed = false;
  N.yfixed = false;
  size_t counter = 0;
  for (size_t j = 0; j <= box->Grid.Ny; j++) {
    for (size_t i = 0; i <= box->Grid.Nx; i++) {
      N.number = counter;
      N.pos.x = (double)i * box->Grid.lx;
      N.pos.y = (double)j * box->Grid.ly;
      box->nodes.push_back(N);
      counter++;
    }
  }

  if (element::nbNodes == 4) {
    // 3 2
    // 0 1
    if (!box->Elem.empty()) box->Elem.clear();
    element E;
    for (size_t j = 0; j < box->Grid.Ny; j++) {
      for (size_t i = 0; i < box->Grid.Nx; i++) {
        E.I[0] = (box->Grid.Nx + 1) * j + i;
        E.I[1] = (box->Grid.Nx + 1) * j + i + 1;
        E.I[2] = (box->Grid.Nx + 1) * (j + 1) + i + 1;
        E.I[3] = (box->Grid.Nx + 1) * (j + 1) + i;
        box->Elem.push_back(E);
      }
    }
  } else if (element::nbNodes == 16) {
    //    13 12 11 10
    // 	  14 3  2  9
    //    15 0  1  8
    //    4  5  6  7
    for (size_t j = 0; j < box->Grid.Ny; j++) {
      for (size_t i = 0; i < box->Grid.Nx; i++) {
        element E;
        E.I[0] = (box->Grid.Nx + 1) * j + i;
        E.I[1] = (box->Grid.Nx + 1) * j + i + 1;
        E.I[2] = (box->Grid.Nx + 1) * (j + 1) + i + 1;
        E.I[3] = (box->Grid.Nx + 1) * (j + 1) + i;
        if (j != 0 && j != box->Grid.Ny - 1 && i != 0 && i != box->Grid.Nx - 1) {
          E.I[4] = (box->Grid.Nx + 1) * (j - 1) + i - 1;
          E.I[5] = (box->Grid.Nx + 1) * (j - 1) + i;
          E.I[6] = (box->Grid.Nx + 1) * (j - 1) + i + 1;
          E.I[7] = (box->Grid.Nx + 1) * (j - 1) + i + 2;

          E.I[8] = (box->Grid.Nx + 1) * j + i + 2;
          E.I[9] = (box->Grid.Nx + 1) * (j + 1) + i + 2;
          E.I[10] = (box->Grid.Nx + 1) * (j + 2) + i + 2;

          E.I[11] = (box->Grid.Nx + 1) * (j + 2) + i + 1;
          E.I[12] = (box->Grid.Nx + 1) * (j + 2) + i;
          E.I[13] = (box->Grid.Nx + 1) * (j + 2) + i - 1;

          E.I[14] = (box->Grid.Nx + 1) * (j + 1) + i - 1;
          E.I[15] = (box->Grid.Nx + 1) * j + i - 1;
        }
        box->Elem.push_back(E);
      }
    }
  } else {
    Logger::error("element::nbNodes = {}! It can only be 4 or 16", element::nbNodes);
  }

  // initial nodes for the first time
  for (size_t i = 0; i < box->nodes.size(); i++) {
    box->liveNodeNum.push_back(i);
  }
}
