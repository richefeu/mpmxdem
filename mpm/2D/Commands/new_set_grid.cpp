#include "new_set_grid.hpp"
#include "../Core/MPMbox.hpp"

void new_set_grid::read(std::istream& is) { is >> lengthX >> lengthY >> spacing; }

void new_set_grid::exec() {

  if (box->shapeFunction == nullptr) {
    Logger::critical("@new_set_grid::exec, ShapeFunction has to be set BEFORE new_set_grid");
    Logger::critical("  It decides whether an element holds 4 or 16 nodes");
    exit(EXIT_FAILURE);
  }
  if (spacing <= 0.0 || lengthX < spacing || lengthY < spacing) {
    Logger::critical("@new_set_grid::exec, invalid grid: lengthX = {}, lengthY = {}, spacing = {}", lengthX, lengthY,
                     spacing);
    exit(EXIT_FAILURE);
  }

  box->Grid.Nx = static_cast<size_t>(floor(lengthX / spacing));
  box->Grid.Ny = static_cast<size_t>(floor(lengthY / spacing));

  // Used when calculating the shape Functions
  box->Grid.lx = spacing;
  box->Grid.ly = spacing;

  if (element::nbNodes != 4 && element::nbNodes != 16) {
    Logger::critical("@new_set_grid::exec, element::nbNodes = {}! It can only be 4 or 16", element::nbNodes);
    exit(EXIT_FAILURE);
  }

  box->buildGrid();
}
