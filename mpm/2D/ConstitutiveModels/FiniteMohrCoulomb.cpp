#include "FiniteMohrCoulomb.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<ConstitutiveModel, FiniteMohrCoulomb> registrar("FiniteMohrCoulomb");

// ==================================================================================
//  2D version of Mohr-Coulomb model
//   - plane strain
//   - take care of the apex-area
//   - DO NOT apply correction on the displacement in case of return to the apex-area
// ==================================================================================

FiniteMohrCoulomb::FiniteMohrCoulomb(double young, double poisson, double frictionAngle, double cohesion,
                                     double dilatancyAngle)
    : Young(young), Poisson(poisson), FrictionAngle(frictionAngle), Cohesion(cohesion), DilatancyAngle(dilatancyAngle) {
  sinFrictionAngle = sin(FrictionAngle);
  sinDilatancyAngle = sin(DilatancyAngle);
  cosFrictionAngle = cos(FrictionAngle);
}

void FiniteMohrCoulomb::read(std::istream& is) {
  is >> Young >> Poisson >> FrictionAngle >> Cohesion >> DilatancyAngle;
  sinFrictionAngle = sin(FrictionAngle);
  sinDilatancyAngle = sin(DilatancyAngle);
  cosFrictionAngle = cos(FrictionAngle);
}

void FiniteMohrCoulomb::updateStrainAndStress(MPMbox& MPM, size_t p) {

  /*
  // Get pointer to the first of the nodes
  int * I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Compute a strain increment (during dt) from the node-velocities
  vec2r vn;
  mat4 dstrain;
  for (int r = 0 ; r < element::nbNodes ; r++) {
          if (MPM.nodes[I[r]].mass > MPM.tolmass) {
                  vn = MPM.nodes[I[r]].q / MPM.nodes[I[r]].mass;
          }
          else continue;
          dstrain.xx += (vn.x * MPM.MP[p].gradN[r].x) * MPM.dt;
          dstrain.xy += 0.5 * (vn.x * MPM.MP[p].gradN[r].y + vn.y * MPM.MP[p].gradN[r].x) * MPM.dt;
          dstrain.yy += (vn.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }
  dstrain.yx = dstrain.xy;
  MPM.MP[p].deltaStrain = dstrain;
  //if (p == 0 and MPM.step % 1 == 0) {std::cout<<"strain at step = "<<MPM.step<<"\n"<<dstrain<<std::endl; double
  delvar; std::cin>>delvar;}


  MPM.MP[p].strain += dstrain;

  */
  mat4 dstrain;
  dstrain = MPM.dt * 0.5 * (MPM.MP[p].velGrad + MPM.MP[p].velGrad.transpose());
  MPM.MP[p].strain += dstrain;
  //      |De11 De12 0   |       |a        Poisson  0  |  with a = 1 - Poisson
  // De = |De12 De22 0   | = f * |Poisson  a        0  |       b = 1 - 2Poisson
  //      |0    0    De33|       |0        0        b/2|   and f = Young / (1 + 2Poisson)
  double a = 1.0 - Poisson;
  double b = (1.0 - 2.0 * Poisson);
  double f = Young / ((1.0 + Poisson) * b);
  double De11 = f * a;
  double De12 = f * Poisson;
  double De22 = De11;
  double De33 = f * 0.5 * b;

  // Trial stress
  MPM.MP[p].stress.xx += De11 * dstrain.xx + De12 * dstrain.yy;
  MPM.MP[p].stress.yy += De12 * dstrain.xx + De22 * dstrain.yy;
  MPM.MP[p].stress.xy += 2 * De33 * dstrain.xy;
  MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;

  double diff_3_1 = sqrt(4.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy +
                         (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy));
  double sum_1_3 = MPM.MP[p].stress.xx + MPM.MP[p].stress.yy;
  double yieldF = diff_3_1 + sum_1_3 * sinFrictionAngle - 2.0 * Cohesion * cosFrictionAngle;

  mat4 deltaPlasticStrain;
  if (yieldF > 0.0) {

    if (MPM.MP[p].plastic == false) MPM.MP[p].plastic = true;
    // Check if sigma is outside the 'apex-area'
    double s = 0.5 * (-sinDilatancyAngle * diff_3_1 + b * sum_1_3) / b;
    double apex = Cohesion * cosFrictionAngle / sinFrictionAngle;

    if (s < apex) {  // okay, we can iterate
      int iter = 0;
      while (iter < 50 and yieldF > 1e-10) {  // Actually a single iteration should be okay	HERE WE HAD || INSTEAD
                                              // OF && AND WAS A SOURCE OF ISSUES

        iter++;

        double diff_xx_yy = MPM.MP[p].stress.xx - MPM.MP[p].stress.yy;
        double div = sqrt((diff_xx_yy) * (diff_xx_yy) + 4.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy);
        double inv_div = 1.0f / div;  // div is not supposed to be null

        double gradfxx = (diff_xx_yy)*inv_div + sinFrictionAngle;
        double gradfyy = -(diff_xx_yy)*inv_div + sinFrictionAngle;
        double gradfxy = 4.0 * MPM.MP[p].stress.xy * inv_div;

        double gradgxx = (diff_xx_yy)*inv_div + sinDilatancyAngle;
        double gradgyy = -(diff_xx_yy)*inv_div + sinDilatancyAngle;
        double gradgxy = 4.0 * MPM.MP[p].stress.xy * inv_div;

        double bottom_lambda = gradfxx * (De11 * gradgxx + De12 * gradgyy) +
                               gradfyy * (De12 * gradgxx + De22 * gradgyy) + gradfxy * De33 * gradgxy;
        double lambda = yieldF / bottom_lambda;

        deltaPlasticStrain.xx = lambda * gradgxx;
        deltaPlasticStrain.yy = lambda * gradgyy;
        deltaPlasticStrain.xy = 0.5 * lambda * gradgxy;
        deltaPlasticStrain.yx = deltaPlasticStrain.xy;

        MPM.MP[p].plasticStrain += deltaPlasticStrain;  // FIXME: INCREMENTED AT EACH ITERATION?

        // Updating total strain for later use
        // MPM.MP[p].elasticStrain = MPM.MP[p].strain - deltaPlasticStrain; /// ??delta??

        // Correcting state of stress
        mat4 delta_sigma_corrector;
        delta_sigma_corrector.xx = De11 * deltaPlasticStrain.xx + De12 * deltaPlasticStrain.yy;
        delta_sigma_corrector.yy = De12 * deltaPlasticStrain.xx + De22 * deltaPlasticStrain.yy;
        delta_sigma_corrector.xy = De33 * deltaPlasticStrain.xy;
        delta_sigma_corrector.yx = delta_sigma_corrector.xy;

        MPM.MP[p].stress -= delta_sigma_corrector;

        // New value of the yield function
        diff_3_1 = sqrt(4.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy +
                        (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy));
        sum_1_3 = MPM.MP[p].stress.xx + MPM.MP[p].stress.yy;
        yieldF = diff_3_1 + sum_1_3 * sinFrictionAngle - 2.0 * Cohesion * cosFrictionAngle;

      }       // end iterations
    } else {  // case in the apex-area
      MPM.MP[p].stress.xx = MPM.MP[p].stress.yy = apex;
      MPM.MP[p].stress.xy = MPM.MP[p].stress.yx = 0.0;
    }
  }
  // if (p == 0 and MPM.step % 1 == 0) {std::cout<<"Stress at step = "<<MPM.step<<"\n"<<MPM.MP[p].stress<<std::endl;
  // double delvar; std::cin>>delvar;}
}
