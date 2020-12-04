#include "set_grid.hpp"

#include <Core/MPMbox.hpp>

#include <factory.hpp>
static Registrar<Command, set_grid> registrar("set_grid");

void set_grid::read(std::istream& is) {
  std::cout << "*****USE 'new_set_grid' INSTEAD OF 'set_grid'*****" << '\n';
  is >> nbElemX >> nbElemY >> lx >> ly;
}

void set_grid::exec() {
  if (box->shapeFunction == nullptr) {
    std::cerr << "@set_grid::exec, ShapeFunction has to be set BEFORE set_grid." << std::endl;
    exit(0);
  }

  box->Grid.Nx = nbElemX;
  box->Grid.Ny = nbElemY;
  box->Grid.lx = lx;
  box->Grid.ly = ly;

  std::cout << "Grid dimensions: x: " << box->Grid.Nx * box->Grid.lx << "\ty: " << box->Grid.Ny * box->Grid.ly
            << std::endl;

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
  int counter = 0;
  for (int j = 0; j <= box->Grid.Ny; j++) {
    for (int i = 0; i <= box->Grid.Nx; i++) {
      N.number = counter;
      N.pos.x = i * box->Grid.lx;
      N.pos.y = j * box->Grid.ly;
      box->nodes.push_back(N);
      counter++;
    }
  }

  if (element::nbNodes == 4) {
    // 3 2
    // 0 1
    if (!box->Elem.empty()) box->Elem.clear();
    element E;
    for (int j = 0; j < box->Grid.Ny; j++) {
      for (int i = 0; i < box->Grid.Nx; i++) {
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
    for (int j = 0; j < box->Grid.Ny; j++) {
      for (int i = 0; i < box->Grid.Nx; i++) {
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
    std::cerr << "element::nbNodes = " << element::nbNodes << "! It can only be 4 or 16." << std::endl;
  }

  // initial nodes for the first time
  for (size_t i = 0; i < box->nodes.size(); i++) {
    box->liveNodeNum.push_back(i);
  }
}
