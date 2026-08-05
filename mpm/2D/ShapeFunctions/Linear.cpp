#include "Linear.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string Linear::getRegistrationName() { return std::string("Linear"); }

Linear::Linear() { element::nbNodes = 4; }

//
// Bilinear interpolation over the 4 nodes of the element, written in the
// reduced coordinates (xi, eta) of the element, both in [-1, 1]:
//
//   N = 1/4 (1 +/- xi)(1 +/- eta)
//
// The signs follow the node numbering of the elements (see Element.hpp):
//
//   I[3] +-------+ I[2]        eta
//        |       |              ^
//        |   .   |              |
//        |       |              +---> xi
//   I[0] +-------+ I[1]
//
void Linear::computeInterpolationValues(MPMbox& MPM, size_t p) {
  MPM.MP[p].e = (size_t)(trunc(MPM.MP[p].pos.x / MPM.Grid.lx) + trunc(MPM.MP[p].pos.y / MPM.Grid.ly) * (double)MPM.Grid.Nx);
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // d(xi)/dx and d(eta)/dy
  double invx = 2.0 / MPM.Grid.lx;
  double invy = 2.0 / MPM.Grid.ly;

  // Position of the Material Point in the reduced coordinates of the element
  double xi = (MPM.MP[p].pos.x - MPM.nodes[I[0]].pos.x) * invx - 1.0;
  double eta = (MPM.MP[p].pos.y - MPM.nodes[I[0]].pos.y) * invy - 1.0;

  double xiM = 0.25 * (1.0 - xi);
  double xiP = 0.25 * (1.0 + xi);
  double etaM = 1.0 - eta;
  double etaP = 1.0 + eta;

  MPM.MP[p].N[0] = xiM * etaM;
  MPM.MP[p].N[1] = xiP * etaM;
  MPM.MP[p].N[2] = xiP * etaP;
  MPM.MP[p].N[3] = xiM * etaP;

  double qx = 0.25 * invx;
  double qy = 0.25 * invy;

  MPM.MP[p].gradN[0].x = -qx * etaM;
  MPM.MP[p].gradN[1].x = qx * etaM;
  MPM.MP[p].gradN[2].x = qx * etaP;
  MPM.MP[p].gradN[3].x = -qx * etaP;

  MPM.MP[p].gradN[0].y = -qy * (1.0 - xi);
  MPM.MP[p].gradN[1].y = -qy * (1.0 + xi);
  MPM.MP[p].gradN[2].y = qy * (1.0 + xi);
  MPM.MP[p].gradN[3].y = qy * (1.0 - xi);
}
