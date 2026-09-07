#include "Scalars.hpp"

#include <cmath>

#include "PostSession.hpp"
#include "ScalarTools.hpp"

void Scalars::read(std::istream &is) {
  is >> filename >> d >> rho_s >> Pmin >> gammaMin >> zzMode;
  if (zzMode == "elastic") {
    is >> Poisson;
  } else if (zzMode != "stored") {
    Logger::warn("@Scalars::read, zzMode '{}' is neither 'stored' nor 'elastic', 'stored' is assumed", zzMode);
    zzMode = "stored";
  }
}

void Scalars::begin() {
  file.open(session->outPath(filename).c_str());
  file << "# Macroscopic mu(I) scalars of the Material Points\n";
  file << "# d = " << d << " m   rho_s = " << rho_s << " kg/m3   Pmin = " << Pmin
       << " Pa   gammaMin = " << gammaMin << "   sigma_zz from '" << zzMode << "'";
  if (zzMode == "elastic") { file << " with nu = " << Poisson; }
  file << '\n';
  file << "# P > 0 in compression (the MPM stress is negated). mu and I are left\n";
  file << "# at 0, and valid at 0, where P <= Pmin or gamma < gammaMin: those\n";
  file << "# points are not measurements of a steady flow.\n";
  file << "# 1:iconf 2:t 3:p(MP number) 4:x 5:y 6:vx 7:vy "
          "8:P 9:tau 10:mu 11:gdot 12:I "
          "13:sig_xx 14:sig_yy 15:sig_zz 16:sig_xy 17:sig_xz 18:sig_yz "
          "19:rho 20:vol 21:mass 22:plastic 23:valid 24:gamma "
          "25:sinPhi 26:tanPhi\n";
}

void Scalars::exec() {
  size_t nbValid   = 0;
  bool allZZareNil = true;

  for (size_t p = 0; p < session->Conf.MP.size(); p++) {
    const MaterialPoint &MP  = session->Conf.MP[p];
    const ProcessedDataMP &D = session->Data[p];

    double szz = D.outOfPlaneStress;
    if (zzMode == "elastic") { szz = Poisson * (D.stress.xx + D.stress.yy); }
    if (szz != 0.0) { allZZareNil = false; }

    const mat9r Sigma = ScalarTools::fullStress(D.stress, szz, D.outOfPlaneShearXZ, D.outOfPlaneShearYZ);

    // The MPM counts a compression as negative, PBC3D as positive. P is
    // negated so that both DEMScalars and this action report the same sign,
    // and can be plotted together.
    const double P    = -Sigma.trace() / 3.0;
    const double tau  = ScalarTools::sqrtJ2(Sigma);
    const double gdot = ScalarTools::shearRate2D(D.velGrad);

    // D.strain holds the deformation gradient F -- see MPMbox::postProcess.
    const double gamma = ScalarTools::equivalentShearStrain(D.strain);

    // Mobilised friction, the other reading of 'mu'. The MPM counts a
    // compression as negative, so the tensor is negated first.
    mat9r Spos          = Sigma;
    Spos                = Spos * (-1.0);
    const double sinPhi = ScalarTools::mobilisedSinPhi(Spos);
    const double tanPhi = ScalarTools::sinPhiToTanPhi(sinPhi);

    double mu = 0.0;
    double I  = 0.0;
    int valid = 0;
    if (P > Pmin && P > 0.0 && gamma >= gammaMin) {
      mu = tau / P;
      if (rho_s > 0.0) { I = gdot * d / sqrt(P / rho_s); }
      valid = 1;
      nbValid++;
    }

    file << session->confNum << ' ' << session->confTime << ' ' << p << ' ' << D.pos.x << ' ' << D.pos.y << ' '
         << D.vel.x << ' ' << D.vel.y << ' ' << P << ' ' << tau << ' ' << mu << ' ' << gdot << ' ' << I << ' '
         << Sigma.xx << ' ' << Sigma.yy << ' ' << Sigma.zz << ' ' << Sigma.xy << ' ' << Sigma.xz
         << ' ' << Sigma.yz << ' ' << D.rho << ' ' << MP.vol << ' '
         << MP.mass << ' ' << (MP.plastic ? 1 : 0) << ' ' << valid << ' ' << gamma << ' ' << sinPhi << ' ' << tanPhi << '\n';
  }

  if (allZZareNil && !warnedAboutFlatZZ && !session->Conf.MP.empty()) {
    warnedAboutFlatZZ = true;
    Logger::warn("@Scalars::exec, sigma_zz is zero for every Material Point. The plane-strain invariants are then "
                 "wrong. MohrCoulomb does not fill MaterialPoint::outOfPlaneStress: use the 'elastic' zzMode, or "
                 "make the model integrate its out-of-plane component");
  }

  if (nbValid == 0 && !session->Conf.MP.empty()) {
    Logger::warn("@Scalars::exec, no Material Point of conf{} passes P > Pmin = {} and gamma >= gammaMin = {}",
                 session->confNum, Pmin, gammaMin);
  }

  file << std::flush;
}

void Scalars::end() {
  if (file.is_open()) { file.close(); }
}
