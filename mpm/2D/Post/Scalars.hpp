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
// Column 27 is divL = tr(L), the volumetric strain rate. It is not used to
// decide 'valid', it is written so that the plot can filter on it: mu(I) is a
// law of steady shear at constant volume, and a Material Point that compacts
// or dilates fast is not on it. |divL| <= 0.1 gdot is the criterion that
// worked best on the double-scale column collapse.
//
// Column 28 is the solid fraction, rho / rho_s -- the second constitutive
// relation of the mu(I) rheology, and the one that says whether the flow is
// steady. It costs nothing: rho is already in the conf-file, so phi(I) needs
// no DEM file at all. It carries meaning only for a double-scale run.
//
// Plane strain is assumed twice, and both are needed:
//   - on the stress, to build the full 3x3 tensor. Without sigma_zz the
//     invariants of a 2D tensor are not those of the real 3D state, and the
//     mu they give is wrong.
//   - on the velocity gradient, with L_zz = 0, for the shear rate.
//
// MohrCoulomb now integrates its out-of-plane component (trial increment,
// plastic corrector and apex return), and set_K0_stress initialises it, so
// 'stored' is the right mode for it as well as for CHCL_DEM, which takes
// sigma_zz straight from the DEM cell. 'elastic' -- nu (sigma_xx + sigma_yy)
// -- is only for conf files written before that fix; it holds while the
// point is elastic and nowhere else.
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
