#include "SinfoniettaClassica.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

// En cours de developpement à l'aide de Quentin.

// This is a 'simple' version of Sinfonietta Classica
// This is a kind of CamClay model. One objective is to extend this model to account for high compressibility.

std::string SinfoniettaClassica::getRegistrationName() { return std::string("SinfoniettaClassica"); }

SinfoniettaClassica::SinfoniettaClassica(double young, double poisson)
    : Young(young), Poisson(poisson), C(young, poisson), kappa(0.0), varphi(0.0), pc0(0.0) {}

void SinfoniettaClassica::read(std::istream& is) {
  double varphiDeg;
  is >> Young >> Poisson >> beta >> beta_p >> kappa >> varphiDeg >> pc0;
  varphi = M_PI * varphiDeg / 180.0;
  C.set(Young, Poisson);
  double sinVarphi = sin(varphi);
  double Z = 6.0 * sinVarphi / (3.0 - sinVarphi);
  z = (9.0 - Z) * (9.0 - Z) / (3.0 - Z * Z + 2.0 * Z * Z * Z / 9.0);
}
void SinfoniettaClassica::write(std::ostream& os) {
  os << Young << ' ' << Poisson << ' ' << beta << ' ' << beta_p << ' ' << kappa << ' ' << varphi << ' ' << pc0 << '\n';
}

void SinfoniettaClassica::init(MaterialPoint& MP) {
  if (MP.hardeningForce == 0.0) {
    MP.hardeningForce = -log(pc0);
    /*
    MP.stress.xx = -pc0;
    MP.stress.yy = -pc0;
    MP.stress.xy = MP.stress.yx = 0.0;
    MP.outOfPlaneStress = -pc0;
    */
  }
}

double SinfoniettaClassica::func_f(mat9r Sigma, double q) {
  // c'est les notations de l'annexe B de la these de Quentin
  double p = -Sigma.trace() / 3.0 + 1e-13;
  mat9r s = Sigma + p * mat9r::unit();
  mat9r xi = s * (1.0 / p);
  double J2xi = inner_product(xi, xi);
  double J3xi = inner_product(xi * xi, xi);

  return (3.0 * beta * (z - 3.0) * (log(p) - q) + 9.0 * (z - 1.0) * J2xi / 4.0 + z * J3xi);
}

mat9r SinfoniettaClassica::func_df_ds(mat9r Sigma, double /*q*/) {
  double p = -Sigma.trace() / 3.0 + epsilon_pressure;
  mat9r s = Sigma + p * mat9r::unit();
  mat9r xi = s * (1.0 / p);
  double J2xi = inner_product(xi, xi);
  double J3xi = inner_product(xi * xi, xi);

  mat9r dev_xi_xi = deviatoric(xi * xi);
  mat9r df_ds =
      -(3.0 * beta * (z - 3.0) / p - 9.0 * (z - 1.0) * J2xi / (2.0 * p) - 3.0 * z * J3xi / p) / 3.0 * mat9r::unit();
  df_ds += (9.0 * (z - 1.0) * xi / (2.0 * p) + 3 * z * dev_xi_xi / p);
  return df_ds;
}

mat9r SinfoniettaClassica::func_dg_ds(mat9r Sigma, double /*q*/) {
  double p = -Sigma.trace() / 3.0 + epsilon_pressure;
  mat9r s = Sigma + p * mat9r::unit();
  mat9r xi = s * (1.0 / p);
  double J2xi = inner_product(xi, xi);
  double J3xi = inner_product(xi * xi, xi);

  mat9r dev_xi_xi = deviatoric(xi * xi);
  mat9r dg_ds = -(9.0 * (z - 3.0) / p - 9.0 * (z - 1.0) * J2xi / (2.0 * p) - 3.0 * z * J3xi / p) / 3.0 * mat9r::unit();
  dg_ds += (9.0 * (z - 1.0) * xi / (2.0 * p) + 3 * z * dev_xi_xi / p);
  return dg_ds;
}

double SinfoniettaClassica::func_df_dq(mat9r /*Sigma*/, double /*q*/) { return 3.0 * beta * (z - 3.0); }

double SinfoniettaClassica::func_h(mat9r Sigma, double q) {
  mat9r dg_ds = func_dg_ds(Sigma, q);
  mat9r dev_dg_ds = deviatoric(dg_ds);
  return -dg_ds.trace() + kappa * sqrt(inner_product(dev_dg_ds, dev_dg_ds));
}

void SinfoniettaClassica::updateStrainAndStress(MPMbox& MPM, size_t p) {
  // Get pointer to the first of the nodes
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Compute a strain increment (during dt) from the node-velocities
  vec2r vn;
  mat4r dstrain;
  for (size_t r = 0; r < element::nbNodes; r++) {
    dstrain.xx += (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy +=
        0.5 * (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].y + MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }
  dstrain.yx = dstrain.xy;
  MPM.MP[p].deltaStrain = dstrain;
  MPM.MP[p].strain += dstrain;

  // clang-format off
  mat9r SigmaTrial(MPM.MP[p].stress.xx, MPM.MP[p].stress.xy, 0.0,
                   MPM.MP[p].stress.yx, MPM.MP[p].stress.yy, 0.0,
                   0.0,                 0.0,                 MPM.MP[p].outOfPlaneStress);

  mat9r dstrain3x3(dstrain.xx, dstrain.xy, 0.0,
                   dstrain.yx, dstrain.yy, 0.0,
                   0.0,        0.0,        0.0);

  SigmaTrial += C.getStress(dstrain3x3);

  double qTrial = MPM.MP[p].hardeningForce;
  mat9r EpTrial(MPM.MP[p].plasticStrain.xx, MPM.MP[p].plasticStrain.xy, 0.0,
                MPM.MP[p].plasticStrain.xx, MPM.MP[p].plasticStrain.xy, 0.0,
                0.0,                        0.0,                        0.0);
  // clang-format on
  double fTrial = func_f(SigmaTrial, qTrial);

  if (fTrial > yield_tol) {
    double yieldF = fTrial;
    mat9r Sigma = SigmaTrial;
    mat9r Ep = EpTrial;
    mat9r incEp;
    double q = qTrial;
    int iter = 0;
    double Hmodulus = 1.0 / beta_p;
    while (iter < 200 && yieldF > yield_tol) {

      iter++;
      mat9r dg_ds = func_dg_ds(Sigma, q);
      mat9r df_ds = func_df_ds(Sigma, q);
      double df_dq = func_df_dq(Sigma, q);
      double h = func_h(Sigma, q);
      double df_C_dg = C.bigDenum(df_ds, dg_ds);

      double d2lambda = yieldF / (df_C_dg + Hmodulus * df_dq * h);

      incEp = d2lambda * dg_ds;
      Ep += incEp;
      Sigma -= C.getStress(incEp);
      q -= d2lambda * Hmodulus * h;
      yieldF = func_f(Sigma, q);
    }

    MPM.MP[p].stress.xx = Sigma.xx;
    MPM.MP[p].stress.xy = MPM.MP[p].stress.yx = Sigma.xy;
    MPM.MP[p].stress.yy = Sigma.yy;
    MPM.MP[p].outOfPlaneStress = Sigma.zz;

    MPM.MP[p].hardeningForce = q;

    MPM.MP[p].plasticStrain.xx = Ep.xx;
    MPM.MP[p].plasticStrain.xy = MPM.MP[p].plasticStrain.yx = Ep.xy;
    MPM.MP[p].plasticStrain.yy = Ep.yy;

  } else {

    MPM.MP[p].stress.xx = SigmaTrial.xx;
    MPM.MP[p].stress.xy = MPM.MP[p].stress.yx = SigmaTrial.xy;
    MPM.MP[p].stress.yy = SigmaTrial.yy;
    MPM.MP[p].outOfPlaneStress = SigmaTrial.zz;
  }
}

double SinfoniettaClassica::getYoung() { return Young; }

double SinfoniettaClassica::getPoisson() { return Poisson; }
