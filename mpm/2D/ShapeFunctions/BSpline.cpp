#include <cstdlib>

#include "BSpline.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

//#include "factory.hpp"
//static Registrar<ShapeFunction, BSpline> registrar("BSpline");
std::string BSpline::getRegistrationName() { return std::string("BSpline"); }

// Reference: Paper Steffen - Analysis and reduction of quadrature errors in mpm
BSpline::BSpline() : TwoThirds(2.0 / 3.0), FourThirds(4.0 / 3.0), OneSixth(1.0 / 6.0) { element::nbNodes = 16; }

void BSpline::computeInterpolationValues(MPMbox& MPM, size_t p) {
  double invL[2];
  invL[0] = 1.0f / MPM.Grid.lx;
  invL[1] = 1.0f / MPM.Grid.ly;

  MPM.MP[p].e = (size_t)(trunc(MPM.MP[p].pos.x * invL[0]) + trunc(MPM.MP[p].pos.y * invL[1]) * (double)MPM.Grid.Nx);
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  std::vector<double> localCoord;
  std::vector<double> Phi;      // we calculate for 1D and then we multiply when we calculate the N vector
  std::vector<double> PhiGrad;  // same procedure as for Phi

  for (int i = 0; i < 16; i++) {
    localCoord.push_back((MPM.MP[p].pos.x - MPM.nodes[I[i]].pos.x) * invL[0]);
    localCoord.push_back((MPM.MP[p].pos.y - MPM.nodes[I[i]].pos.y) * invL[1]);
  }

  // careful when finding the gradient. dont derive using local coordinates.
  // instead use (MPpos-nodepos)/cellLength. otherwise you'll miss a cellLength division!
  int iL = 0;
  for (size_t i = 0; i < localCoord.size(); i++) {
    double x = localCoord[i];

    double absx = fabs(x);
    if (absx >= 0.0 && absx < 1.0) {
      Phi.push_back(0.5 * absx * absx * absx - x * x + TwoThirds);
      PhiGrad.push_back(x * (1.5 * absx - 2.0) * invL[iL]);
    } else if (absx < 2.0) {
      Phi.push_back(-OneSixth * absx * absx * absx + x * x - 2.0 * absx + FourThirds);
      PhiGrad.push_back(-x * (absx - 2.0) * (absx - 2.0) * invL[iL] / (2.0 * absx));
    } else {
      std::cerr << "@BSpline::computeInterpolationValues, we shouldn't be landing here!" << std::endl;
      std::cerr << "MPs could be outside the grid." << std::endl;
      std::cerr << "Check MP number " << p << ", at position " << MPM.MP[p].pos << std::endl;
      //exit(EXIT_FAILURE);
    }
    iL = 1 - iL;  // this will generate 1 0 1 0 and so on, to switch between lx and ly
  }

  // Calculating N and gradN
  for (int i = 0; i < 16; i++) {
    int ix = 2 * i;
    int iy = ix + 1;
    MPM.MP[p].N[i] = Phi[ix] * Phi[iy];
    MPM.MP[p].gradN[i].x = PhiGrad[ix] * Phi[iy];
    MPM.MP[p].gradN[i].y = Phi[ix] * PhiGrad[iy];
  }
}
