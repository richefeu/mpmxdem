#pragma once

#include <fstream>
#include <string>

#include "PostProcessor.hpp"

//
// Macroscopic mu(I) scalars, taken from the state of the Material Points
// themselves. This is the single-scale counterpart of DEMScalars: it works
// for any constitutive model -- MohrCoulomb in particular -- and produces the
// same columns P, tau, mu, gdot and I, so that a classical run and a
// double-scale run can be compared on the same axes.
//
//     Post Scalars <fileName> <d> <rho_s> <Pmin> <gammaMin> <zzMode> [<Poisson>]
//
// with
//
//   d, rho_s   grain diameter and grain density, used to form I. A continuum
//              model has no grains, so these are given by hand; take those of
//              the DEM sample the run is to be compared with.
//   Pmin       Material Points with P <= Pmin get mu = I = 0 and are meant to
//              be dropped when plotting. A free-surface or airborne point has
//              a vanishing pressure, and mu = tau/P would be meaningless
//              noise there.
//   gammaMin   likewise for points whose accumulated shear strain is below
//              gammaMin. mu(I) is a steady-flow law, reached once the
//              material has been sheared by an amount of order one; a point
//              that has barely deformed is still on its elastic branch and
//              says nothing about the rheology. 0 keeps everything. See
//              ScalarTools::equivalentShearStrain for what is measured, and
//              for what it does not measure.
//   zzMode     how the out-of-plane stress sigma_zz is obtained:
//                'stored'  the value carried by the Material Point.
//                'elastic' nu (sigma_xx + sigma_yy), the plane-strain elastic
//                          expression; then <Poisson> must follow.
//
// Plane strain is assumed twice, and both are needed:
//   - on the stress, to build the full 3x3 tensor. Without sigma_zz the
//     invariants of a 2D tensor are not those of the real 3D state, and the
//     mu they give is wrong.
//   - on the velocity gradient, with L_zz = 0, for the shear rate.
//
// /!\ MohrCoulomb never fills MaterialPoint::outOfPlaneStress -- it is the
// only model of ConstitutiveModels/ that does not. With 'stored' a
// Mohr-Coulomb run therefore gives sigma_zz = 0, which is NOT plane strain
// and biases P and tau. The action says so rather than pretending otherwise.
// Use 'elastic' as a stopgap, knowing it only holds while the point is
// elastic, or teach MohrCoulomb to integrate its out-of-plane component.
//
struct Scalars : public PostProcessor {
  void read(std::istream &is);
  void begin();
  void exec();
  void end();

private:
  std::string filename;
  double d{1.0};
  double rho_s{1.0};
  double Pmin{0.0};
  double gammaMin{0.0};
  std::string zzMode{"stored"};
  double Poisson{0.0};

  std::ofstream file;
  bool warnedAboutFlatZZ{false};
};
