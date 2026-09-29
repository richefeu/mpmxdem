#pragma once

#include <fstream>
#include <string>

#include "PostProcessor.hpp"

//
// Micro-scale scalars of the DEM cells, for a double-scale (MPMxDEM)
// computation. This is the entry point for the mu(I) analysis.
//
//     Post DEMScalars <fileName> <gammaMin>
//
// Writes a single file holding one line per (configuration, tracked Material
// Point). For every Material Point p, the DEM configuration written by
// MPMbox::run() in 'DEM_MP<p>/conf<N>' is loaded; points that were not
// tracked simply have no such file and are skipped.
//
// Quantities that need saying:
//
//   P     = tr(Sig)/3, the mean pressure of the DEM cell. PBC3D counts a
//           compression as positive, so P > 0 in a flowing granular column.
//   tau   = sqrt(J2) = sqrt(0.5 dev(Sig):dev(Sig))
//   mu    = tau / P, the effective friction of the cell
//   gdot  = sqrt(2 dev(D):dev(D)), the shear rate, with D the symmetric part
//           of a velocity gradient. Two of them are reported: the one the MPM
//           sees at the Material Point (smoothed velGrad, plane strain, so
//           L_zz = 0) and the one the periodic cell actually underwent
//           (Cell.velGrad, which is genuinely 3D).
//   I     = gdot d / sqrt(P / rho_s), with d = 2 Rmean the mean grain
//           diameter and rho_s the grain density. Reported from the cell
//           shear rate. Left at 0 when P <= 0.
//   gamma = accumulated shear strain of the Material Point carrying the cell,
//           from its deformation gradient (ScalarTools::equivalentShearStrain).
//           Cells with gamma < gammaMin get valid = 0: mu(I) is a steady-flow
//           law, and a cell that has barely been sheared has not reached it.
//           gammaMin = 0 keeps everything.
//
struct DEMScalars : public PostProcessor {
  void read(std::istream &is);
  void begin();
  void exec();
  void end();

private:
  std::string filename;
  double gammaMin{0.0};
  std::ofstream file;
};
