#include "VonMisesElastoPlasticity.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string VonMisesElastoPlasticity::getRegistrationName() { return std::string("VonMisesElastoPlasticity"); }

// ==== VonMisesElastoPlasticity =================================================

VonMisesElastoPlasticity::VonMisesElastoPlasticity(double young, double poisson, double plasticYieldStress)
    : Young(young), Poisson(poisson), PlasticYieldStress(plasticYieldStress) {}

void VonMisesElastoPlasticity::read(std::istream& is) { is >> Young >> Poisson >> PlasticYieldStress; }

void VonMisesElastoPlasticity::write(std::ostream& os) {
  os << Young << ' ' << Poisson << ' ' << PlasticYieldStress << '\n';
}

void VonMisesElastoPlasticity::updateStrainAndStress(MPMbox& MPM, size_t p) {
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  vec2r vn;
  mat4r dstrain;
  for (size_t r = 0; r < element::nbNodes; r++) {
    if (MPM.nodes[I[r]].mass > MPM.tolmass) {
      vn = MPM.nodes[I[r]].q / MPM.nodes[I[r]].mass;
    } else {
      continue;
    }

    dstrain.xx += (vn.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy += 0.5 * (vn.x * MPM.MP[p].gradN[r].y + vn.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (vn.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }
  dstrain.yx = dstrain.xy;

  // Update strain
  MPM.MP[p].strain += dstrain;
  MPM.MP[p].deltaStrain = dstrain;

  int N = 1;

  mat4r deltastrain;

  deltastrain.xx = dstrain.xx / N;
  deltastrain.yy = dstrain.yy / N;
  deltastrain.xy = deltastrain.yx = dstrain.xy / N;

  mat4r gradg;
  mat4r gradf;
  mat4r stressCorrection;
  double yieldF;

  for (int i = 1; i <= N; ++i) {
    double a = 1.0 - Poisson;
    double b = 1.0 - 2.0 * Poisson;
    double f = Young / ((1.0 + Poisson) * b);
    MPM.MP[p].stress.xx += f * (a * deltastrain.xx + Poisson * deltastrain.yy);
    MPM.MP[p].stress.yy += f * (a * deltastrain.yy + Poisson * deltastrain.xx);
    MPM.MP[p].stress.xy += f * b * deltastrain.xy;
    MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;

    // from "basic computational plasticity" p. 21
    yieldF =
        sqrt((3.0 * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy)) / 4.0 +
             3.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy) -
        PlasticYieldStress;

    if (yieldF >= 0.0) {
      gradf.xx =
          ((3.0 * MPM.MP[p].stress.xx) / 2.0 - (3.0 * MPM.MP[p].stress.yy) / 2.0) /
          (2.0 *
           sqrt((3.0 * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy)) /
                    4.0 +
                3.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy));
      gradf.yy =
          -((3.0 * MPM.MP[p].stress.xx) / 2.0 - (3.0 * MPM.MP[p].stress.yy) / 2.0) /
          (2.0 *
           sqrt((3.0 * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy)) /
                    4.0 +
                3.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy));
      gradf.xy =
          (3.0 * MPM.MP[p].stress.xy) /
          sqrt((3.0 * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy)) / 4.0 +
               3.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy);

      double bottom_lambda = f * (gradf.xx * (gradf.xx * (1.0 - Poisson) + gradf.yy * Poisson) +
                                  gradf.yy * (gradf.xx * Poisson + gradf.yy * (1.0 - Poisson)) +
                                  gradf.xy * gradf.xy * (1.0 - 2.0 * Poisson) / 2.0);

      double lambdadot = yieldF / bottom_lambda;

      MPM.MP[p].plasticStrain.xx = lambdadot * gradf.xx;
      MPM.MP[p].plasticStrain.yy = lambdadot * gradf.yy;
      MPM.MP[p].plasticStrain.xy = lambdadot * gradf.xy;
      MPM.MP[p].plasticStrain.yx = MPM.MP[p].plasticStrain.xy;

      stressCorrection.xx = f * (MPM.MP[p].plasticStrain.xx * (1.0 - Poisson) + MPM.MP[p].plasticStrain.yy * Poisson);
      stressCorrection.yy = f * (MPM.MP[p].plasticStrain.xx * Poisson + MPM.MP[p].plasticStrain.yy * (1.0 - Poisson));
      stressCorrection.xy = f * (MPM.MP[p].plasticStrain.xy * (1.0 - 2.0 * Poisson));
      stressCorrection.yx = stressCorrection.xy;

      MPM.MP[p].stress.xx = MPM.MP[p].stress.xx - stressCorrection.xx;
      MPM.MP[p].stress.yy = MPM.MP[p].stress.yy - stressCorrection.yy;
      MPM.MP[p].stress.xy = MPM.MP[p].stress.xy - stressCorrection.xy;
      MPM.MP[p].stress.yx = MPM.MP[p].stress.yx - stressCorrection.yx;

      yieldF =
          sqrt((3.0 * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy)) / 4.0 +
               3 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy) -
          PlasticYieldStress;
    }
  }
}
double VonMisesElastoPlasticity::getYoung() { return Young; }

double VonMisesElastoPlasticity::getPoisson() { return Poisson; }
