#include "new_set_grid.hpp"
#include "../Core/MPMbox.hpp"

void new_set_grid::read(std::istream& is) { is >> lengthX >> lengthY >> spacing; }

void new_set_grid::exec() {

  if (box->shapeFunction == nullptr) {
    std::cerr << "@new_set_grid::exec, ShapeFunction has to be set BEFORE set_grid." << std::endl;
    exit(0);
  }

  box->Grid.Nx = static_cast<int>(floor(lengthX / spacing));
  box->Grid.Ny = static_cast<int>(floor(lengthY / spacing));

  // Used when calculating the shape Functions
  box->Grid.lx = spacing;
  box->Grid.ly = spacing;

  // Create the nodes and set their positions
  if (!box->nodes.empty()) box->nodes.clear();
  node N;
  N.mass = 0.0;
  N.q.reset();
  N.f.reset();
  N.fb.reset();
  N.xfixed = false;
  N.yfixed = false;
  int counter = 0;

  for (size_t j = 0; j <= box->Grid.Ny; j++) {
    for (size_t i = 0; i <= box->Grid.Nx; i++) {
      N.number = counter;
      N.pos.x = (double)i * spacing;
      N.pos.y = (double)j * spacing;
      counter++;
      box->nodes.push_back(N);
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
    std::cerr << "element::nbNodes = " << element::nbNodes << "! It can only be 4 or 16." << std::endl;
  }

  // initial nodes for the first time
  for (size_t i = 0; i < box->nodes.size(); i++) {
    box->liveNodeNum.push_back(i);
  }
}
