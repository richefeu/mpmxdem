#include "DEMScalars.hpp"

#include <cmath>

#include "PostSession.hpp"
#include "ScalarTools.hpp"

#include "PBC3D.hpp"

using ScalarTools::shearRate;
using ScalarTools::shearRate2D;
using ScalarTools::sqrtJ2;

void DEMScalars::read(std::istream &is) { is >> filename >> gammaMin; }

void DEMScalars::begin() {
  file.open(session->outPath(filename).c_str());
  file << "# Micro-scale scalars of the DEM cells\n";
  file << "# gammaMin = " << gammaMin << "\n";
  file << "# 1:iconf 2:t 3:p(MP number) 4:x 5:y "
          "6:nbParticles 7:nbActiveInteractions 8:nbBonds "
          "9:Vcell 10:Vsolid 11:solidFraction 12:Rmean 13:rho_s "
          "14:P 15:tau 16:mu 17:gdot_cell 18:gdot_MP 19:I "
          "20:Sig_xx 21:Sig_yy 22:Sig_zz 23:Sig_xy 24:Sig_xz 25:Sig_yz "
          "26:VelMean 27:VelVar 28:gamma 29:valid 30:sinPhi 31:tanPhi "
          "32:tau_ps 33:mu_ps\n";
}

void DEMScalars::exec() {
  PBC3Dbox dem;
  size_t nbLoaded = 0;

  for (size_t p = 0; p < session->Conf.MP.size(); p++) {
    if (!session->loadDEMConf(p, dem)) { continue; }
    nbLoaded++;

    const double Vcell = fabs(dem.Cell.h.det());
    const double nu    = (Vcell > 0.0) ? dem.Vsolid / Vcell : 0.0;

    const double P   = dem.Sig.trace() / 3.0;
    const double tau = sqrtJ2(dem.Sig);

    // Same cell, but with the out-of-plane shears removed: what a plane-strain
    // continuum would be able to show. P is unchanged (the trace is), only the
    // deviatoric invariant drops. This is the column to use when comparing
    // with a Mohr-Coulomb run, which cannot develop sigma_xz or sigma_yz at
    // all -- see the note in Doc/SyntaxMPMpost.md.
    const double tau_ps = sqrtJ2(ScalarTools::planeStrainProjection(dem.Sig));

    const double gdotCell = shearRate(dem.Cell.velGrad);
    const double gdotMP   = shearRate2D(session->Data[p].velGrad);

    // Data[p].strain holds the deformation gradient F of the Material Point
    // that carries this cell -- see MPMbox::postProcess.
    const double gamma = ScalarTools::equivalentShearStrain(session->Data[p].strain);

    // PBC3D already counts a compression as positive, no sign to flip here.
    const double sinPhi = ScalarTools::mobilisedSinPhi(dem.Sig);
    const double tanPhi = ScalarTools::sinPhiToTanPhi(sinPhi);

    // mu and I are left at zero when the cell says nothing about a steady
    // flow, so that a plot filtering on them drops those points.
    const double d = 2.0 * dem.Rmean;
    double mu      = 0.0;
    double mu_ps   = 0.0;
    double I       = 0.0;
    int valid      = 0;
    if (P > 0.0 && gamma >= gammaMin) {
      mu    = tau / P;
      mu_ps = tau_ps / P;
      if (dem.density > 0.0) { I = gdotCell * d / sqrt(P / dem.density); }
      valid = 1;
    }

    file << session->confNum << ' ' << session->confTime << ' ' << p << ' ' << session->Conf.MP[p].pos.x << ' '
         << session->Conf.MP[p].pos.y << ' ' << dem.Particles.size() << ' ' << dem.nbActiveInteractions << ' '
         << dem.nbBonds << ' ' << Vcell << ' ' << dem.Vsolid << ' ' << nu << ' ' << dem.Rmean << ' ' << dem.density
         << ' ' << P << ' ' << tau << ' ' << mu << ' ' << gdotCell << ' ' << gdotMP << ' ' << I << ' ' << dem.Sig.xx
         << ' ' << dem.Sig.yy << ' ' << dem.Sig.zz << ' ' << dem.Sig.xy << ' ' << dem.Sig.xz << ' ' << dem.Sig.yz << ' '
         << dem.VelMean << ' ' << dem.VelVar << ' ' << gamma << ' ' << valid << ' ' << sinPhi << ' ' << tanPhi
         << ' ' << tau_ps << ' ' << mu_ps << '\n';
  }

  if (nbLoaded == 0) {
    Logger::warn("@DEMScalars::exec, no DEM_MP<p>/conf{} file found in '{}'. Either the computation has no double "
                 "scale, or no Material Point was tracked (see 'select_tracked_MP')",
                 session->confNum, session->sourceFolder);
  }

  file << std::flush;
}

void DEMScalars::end() {
  if (file.is_open()) { file.close(); }
}
