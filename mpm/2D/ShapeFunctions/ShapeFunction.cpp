#include "ShapeFunction.hpp"

#include <cmath>

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

ShapeFunction::~ShapeFunction() {}

//
// Locate the element that holds the Material Point p, and store its number in
// MPM.MP[p].e.
//
// The grid is regular, so the element is found by simple truncation:
//
//   e = i + j * Nx     with   i = trunc(x / lx)   and   j = trunc(y / ly)
//
// The valid ranges are i in [0, Nx-1] and j in [0, Ny-1], hence x in [0, Nx*lx[
// and y in [0, Ny*ly[ -- the upper bounds are excluded, because a point sitting
// exactly on the right (or top) border would be attributed to an element that
// does not exist.
//
// A Material Point that leaves the grid is a modelling error, not something the
// solver can carry on with: the element number would be out of range (or, for a
// negative position, a huge number after the conversion to size_t) and
// MPM.Elem[e] would read anywhere in memory. The computation is therefore
// stopped with a message that says which point, where, and when.
//
// The viewer is treated differently: refusing to open a conf-file because one
// point is misplaced would not help anybody, so the index is clamped to the
// closest element and a warning is issued.
//
void ShapeFunction::locateElement(MPMbox& MPM, size_t p) {
  const double W = (double)MPM.Grid.Nx * MPM.Grid.lx;
  const double H = (double)MPM.Grid.Ny * MPM.Grid.ly;
  double x = MPM.MP[p].pos.x;
  double y = MPM.MP[p].pos.y;

  // Negated form on purpose: it also catches NaN positions, for which every
  // direct comparison would be false.
  if (!(x >= 0.0 && x < W) || !(y >= 0.0 && y < H)) {
    if (MPM.computationMode == true) {
      Logger::critical("@ShapeFunction::locateElement, the Material Point {} has left the grid", p);
      Logger::critical("  position = ({}, {}), velocity = ({}, {})", x, y, MPM.MP[p].vel.x, MPM.MP[p].vel.y);
      Logger::critical("  grid     = [0, {}] x [0, {}]  ({} x {} elements of {} x {})", W, H, MPM.Grid.Nx, MPM.Grid.Ny,
                       MPM.Grid.lx, MPM.Grid.ly);
      Logger::critical("  step {}, t = {}", MPM.step, MPM.t);
      Logger::critical("  Enlarge the grid, reduce the time step, or hold the material with an obstacle.");
      exit(EXIT_FAILURE);
    }
    Logger::warn("@ShapeFunction::locateElement, the Material Point {} at ({}, {}) is outside the grid "
                 "[0, {}] x [0, {}]; it is drawn at the closest element",
                 p, x, y, W, H);
  }

  // Clamping the coordinates rather than the indices keeps the conversion to
  // size_t well defined, even for a NaN position.
  if (!(x >= 0.0)) { x = 0.0; }
  if (!(x < W)) { x = W - 0.5 * MPM.Grid.lx; }
  if (!(y >= 0.0)) { y = 0.0; }
  if (!(y < H)) { y = H - 0.5 * MPM.Grid.ly; }

  size_t i = (size_t)trunc(x / MPM.Grid.lx);
  size_t j = (size_t)trunc(y / MPM.Grid.ly);
  // Last guard: for a position just below the border, the division alone can
  // round up to Nx (or Ny).
  if (i >= MPM.Grid.Nx) { i = MPM.Grid.Nx - 1; }
  if (j >= MPM.Grid.Ny) { j = MPM.Grid.Ny - 1; }

  MPM.MP[p].e = i + j * MPM.Grid.Nx;
}
