#include <cstdlib>

#include "BSpline.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string BSpline::getRegistrationName() { return std::string("BSpline"); }

// Reference: Paper Steffen - Analysis and reduction of quadrature errors in mpm
BSpline::BSpline() : TwoThirds(2.0 / 3.0), FourThirds(4.0 / 3.0), OneSixth(1.0 / 6.0) { element::nbNodes = 16; }

//
// The B-spline is a tensor product: N_k = Phi(x_k) * Phi(y_k). The 16 nodes of
// the patch only take FOUR distinct positions in each direction -- the offsets
// above all belong to {-1, 0, 1, 2} -- so there are 4 + 4 = 8 distinct factors,
// not 32. The previous version evaluated the cubic 32 times per Material Point,
// with a branch each time; this one evaluates it 8 times, and the branch is then
// decided by the offset alone, hence perfectly predicted.
//
// The reduced coordinates are computed from the element indices rather than read
// from the node positions: the grid is regular by construction, so
// node_k.pos.x = (ie + dxOff[k]) * lx, and 16 scattered reads of a 156-byte node
// are saved. Measured on the reference benchmark, this phase went from 0.279 s
// to 0.095 s. See Doc/OPTIM.md.
//
void BSpline::computeInterpolationValues(MPMbox& MPM, size_t p) {
  const double invLx = 1.0 / MPM.Grid.lx;
  const double invLy = 1.0 / MPM.Grid.ly;

  locateElement(MPM, p);

  const size_t ie = MPM.MP[p].e % MPM.Grid.Nx;
  const size_t je = MPM.MP[p].e / MPM.Grid.Nx;

  // The outer ring of the patch is guaranteed to exist: MPMbox::buildGrid puts a
  // ghost ring of nodes around the element grid as soon as the elements hold 16
  // nodes (defect A3). The only way to get here without it is to have built the
  // grid before the shape function was known, which the grid commands refuse.
  if (MPM.Grid.pad < 1) {
    Logger::critical("@BSpline::computeInterpolationValues, the node grid has no ghost ring");
    Logger::critical("  The B-splines read one node beyond each element; the grid has to be built");
    Logger::critical("  after 'ShapeFunction BSpline', so that MPMbox::buildGrid knows to pad it.");
    exit(EXIT_FAILURE);
  }

  // Reduced position inside the element, both in [0, 1[
  const double xiR = MPM.MP[p].pos.x * invLx - (double)ie;
  const double etaR = MPM.MP[p].pos.y * invLy - (double)je;

  // The four distinct factors of each direction. The index k stands for the
  // offset k-1, that is -1, 0, 1 and 2.
  double phix[4], dphix[4], phiy[4], dphiy[4];
  for (int k = 0; k < 4; k++) {
    const double off = (double)(k - 1);

    // Direction x. For off = -1 and 2, |x| lies in [1, 2]; for off = 0 and 1, in
    // [0, 1]. The two expressions agree at |x| = 1, so the boundary case is
    // harmless whichever branch is taken.
    {
      const double x = xiR - off;
      const double absx = fabs(x);
      if (absx < 1.0) {
        phix[k] = 0.5 * absx * absx * absx - x * x + TwoThirds;
        dphix[k] = x * (1.5 * absx - 2.0) * invLx;
      } else {
        phix[k] = -OneSixth * absx * absx * absx + x * x - 2.0 * absx + FourThirds;
        dphix[k] = -x * (absx - 2.0) * (absx - 2.0) * invLx / (2.0 * absx);
      }
    }

    // Direction y
    {
      const double y = etaR - off;
      const double absy = fabs(y);
      if (absy < 1.0) {
        phiy[k] = 0.5 * absy * absy * absy - y * y + TwoThirds;
        dphiy[k] = y * (1.5 * absy - 2.0) * invLy;
      } else {
        phiy[k] = -OneSixth * absy * absy * absy + y * y - 2.0 * absy + FourThirds;
        dphiy[k] = -y * (absy - 2.0) * (absy - 2.0) * invLy / (2.0 * absy);
      }
    }
  }

  for (int i = 0; i < 16; i++) {
    const int kx = element::dxOff[i] + 1;
    const int ky = element::dyOff[i] + 1;
    MPM.N(p)[i] = phix[kx] * phiy[ky];
    MPM.gradN(p)[i].x = dphix[kx] * phiy[ky];
    MPM.gradN(p)[i].y = phix[kx] * dphiy[ky];
  }
}
