#include "MohrCoulomb.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string MohrCoulomb::getRegistrationName() {
  return std::string("MohrCoulomb");
}

// ==================================================================================
//  2D version of Mohr-Coulomb model (elasto-plastic without hardening)
//   - plane strain
//   - take care of the apex-area
//   - DO NOT apply correction on the displacement in case of return to the apex-area
//
//  The yield function and the plastic potential are written on the in-plane
//  principal stresses only. This amounts to assuming that sigma_zz is the
//  INTERMEDIATE principal stress -- the usual plane-strain situation, but not
//  something the model checks. sigma_zz is integrated (see
//  updateStrainAndStress) so that MaterialPoint::outOfPlaneStress carries a
//  meaningful value, without any of it feeding back into the in-plane
//  response.
// ==================================================================================

MohrCoulomb::MohrCoulomb(double young, double poisson, double frictionAngle, double cohesion, double dilatancyAngle)
    : Young(young), Poisson(poisson), FrictionAngle(frictionAngle), Cohesion(cohesion), DilatancyAngle(dilatancyAngle) {
  sinFrictionAngle  = sin(FrictionAngle);
  sinDilatancyAngle = sin(DilatancyAngle);
  cosFrictionAngle  = cos(FrictionAngle);
}

void MohrCoulomb::read(std::istream &is) {
  is >> Young >> Poisson >> FrictionAngle >> Cohesion >> DilatancyAngle;
  sinFrictionAngle  = sin(FrictionAngle);
  sinDilatancyAngle = sin(DilatancyAngle);
  cosFrictionAngle  = cos(FrictionAngle);
}

void MohrCoulomb::write(std::ostream &os) {
  os << Young << ' ' << Poisson << ' ' << FrictionAngle << ' ' << Cohesion << ' ' << DilatancyAngle << '\n';
}

double MohrCoulomb::getYoung() {
  return Young;
}

double MohrCoulomb::getPoisson() {
  return Poisson;
}

void MohrCoulomb::updateStrainAndStress(MPMbox &MPM, size_t p) {
  // Get pointer to the first of the nodes
  size_t *I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Compute a strain increment (during dt) from the node-velocities
  // vec2r vn;
  mat4r dstrain{};
  const vec2r *gNp = MPM.gradN(p);
  for (size_t r = 0; r < element::nbNodes; r++) {
    dstrain.xx += (MPM.nodes[I[r]].vel.x * gNp[r].x) * MPM.dt;
    dstrain.xy +=
        0.5 * (MPM.nodes[I[r]].vel.x * gNp[r].y + MPM.nodes[I[r]].vel.y * gNp[r].x) * MPM.dt;
    dstrain.yy += (MPM.nodes[I[r]].vel.y * gNp[r].y) * MPM.dt;
  }
  dstrain.yx = dstrain.xy;

  MPM.MP[p].deltaStrain = dstrain; // store it for processing

  MPM.MP[p].strain += dstrain; // increment the strain

  //      |De11 De12 0   |       |a        Poisson  0  |        with a = 1 - Poisson
  // De = |De12 De22 0   | = f * |Poisson  a        0  |             b = 1 - 2Poisson
  //      |0    0    De33|       |0        0        b/2|         and f = Young / (1 + 2Poisson)
  double a    = 1.0 - Poisson;
  double b    = (1.0 - 2.0 * Poisson);
  double f    = Young / ((1.0 + Poisson) * b);
  double De11 = f * a;
  double De12 = f * Poisson;
  double De22 = De11;
  double De33 = f * b;

  // Trial stress
  MPM.MP[p].stress.xx += De11 * dstrain.xx + De12 * dstrain.yy;
  MPM.MP[p].stress.yy += De12 * dstrain.xx + De22 * dstrain.yy;
  MPM.MP[p].stress.xy += De33 * dstrain.xy;
  MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;

  // Out-of-plane component of the trial stress. Plane strain means
  // dstrain_zz = 0, so the third line of the 3D elastic matrix leaves only
  //
  //     dsigma_zz = De12 (dstrain_xx + dstrain_yy)
  //
  // De12 being the Lame coefficient lambda. Note that sigma_zz takes no part
  // in what follows: the yield function and the plastic potential are written
  // on the in-plane principal stresses alone, so carrying it changes nothing
  // to the in-plane response. It only makes the stress state a complete 3D
  // tensor -- which post-processing needs, since the invariants of the 2D
  // tensor alone are not those of the real state.
  MPM.MP[p].outOfPlaneStress += De12 * (dstrain.xx + dstrain.yy);

  double diff_3_1 = sqrt(4.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy +
                         (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy));
  double sum_1_3  = MPM.MP[p].stress.xx + MPM.MP[p].stress.yy;
  double yieldF   = diff_3_1 + sum_1_3 * sinFrictionAngle - 2.0 * Cohesion * cosFrictionAngle;

  mat4r deltaPlasticStrain{};
  if (yieldF > 0.0) {

    if (MPM.MP[p].plastic == false) MPM.MP[p].plastic = true;

    // Check if sigma is outside the 'apex-area'
    double s    = 0.5 * (-sinDilatancyAngle * diff_3_1 + b * sum_1_3) / b;
    double apex = Cohesion * cosFrictionAngle / sinFrictionAngle;

    if (s < apex) { // okay, we can iterate
      int iter = 0;
      while (iter < 50 && yieldF > 1e-10) { // Actually a single iteration should be okay	HERE WE HAD || INSTEAD
                                            // OF && AND WAS A SOURCE OF ISSUES

        iter++;

        double diff_xx_yy = MPM.MP[p].stress.xx - MPM.MP[p].stress.yy;
        double div        = sqrt(diff_xx_yy * diff_xx_yy + 4.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy);
        double inv_div    = 1.0f / div; // div is not supposed to be null

        double gradfxx = diff_xx_yy * inv_div + sinFrictionAngle;
        double gradfyy = -diff_xx_yy * inv_div + sinFrictionAngle;
        double gradfxy = 4.0 * MPM.MP[p].stress.xy * inv_div;

        double gradgxx = diff_xx_yy * inv_div + sinDilatancyAngle;
        double gradgyy = -diff_xx_yy * inv_div + sinDilatancyAngle;
        double gradgxy = 4.0 * MPM.MP[p].stress.xy * inv_div;

        double bottom_lambda = gradfxx * (De11 * gradgxx + De12 * gradgyy) +
                               gradfyy * (De12 * gradgxx + De22 * gradgyy) + gradfxy * De33 * gradgxy;
        double lambda        = yieldF / bottom_lambda;

        deltaPlasticStrain.xx = lambda * gradgxx;
        deltaPlasticStrain.yy = lambda * gradgyy;
        deltaPlasticStrain.xy = 0.5 * lambda * gradgxy;
        deltaPlasticStrain.yx = deltaPlasticStrain.xy;
        MPM.MP[p].plasticStrain += deltaPlasticStrain; // FIXME: INCREMENTED AT EACH ITERATION?

        // Correcting state of stress
        mat4r delta_sigma_corrector;
        delta_sigma_corrector.xx = De11 * deltaPlasticStrain.xx + De12 * deltaPlasticStrain.yy;
        delta_sigma_corrector.yy = De12 * deltaPlasticStrain.xx + De22 * deltaPlasticStrain.yy;
        delta_sigma_corrector.xy = De33 * deltaPlasticStrain.xy;
        delta_sigma_corrector.yx = delta_sigma_corrector.xy;

        MPM.MP[p].stress -= delta_sigma_corrector;

        // The plastic potential is written on the in-plane principal stresses
        // only, so it does not depend on sigma_zz: the plastic strain has no
        // zz component, and the out-of-plane corrector reduces to the Lame
        // term lambda (deps^p_xx + deps^p_yy). With deps_zz = deps^p_zz = 0,
        // the elastic part of the out-of-plane strain stays zero, as plane
        // strain requires.
        MPM.MP[p].outOfPlaneStress -= De12 * (deltaPlasticStrain.xx + deltaPlasticStrain.yy);

        // saving the plastic stress to a mat4 variable
        MPM.MP[p].stressCorrection += delta_sigma_corrector;

        // New value of the yield function
        diff_3_1 = sqrt(4.0 * MPM.MP[p].stress.xy * MPM.MP[p].stress.xy +
                        (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy) * (MPM.MP[p].stress.xx - MPM.MP[p].stress.yy));
        sum_1_3  = MPM.MP[p].stress.xx + MPM.MP[p].stress.yy;
        yieldF   = diff_3_1 + sum_1_3 * sinFrictionAngle - 2.0 * Cohesion * cosFrictionAngle;

      } // end iterations
    } else { // case in the apex-area
      // The apex of the Mohr-Coulomb cone is the isotropic tension
      // c cos(phi)/sin(phi): all three principal stresses are equal there, so
      // the out-of-plane component takes the same value as the in-plane ones.
      MPM.MP[p].stress.xx = MPM.MP[p].stress.yy = apex;
      MPM.MP[p].stress.xy = MPM.MP[p].stress.yx = 0.0;
      MPM.MP[p].outOfPlaneStress               = apex;
    }
  } // end if (yieldD > 0.0)
}

void MohrCoulomb::init(MaterialPoint &MP) {
  MP.isDoubleScale = false;
}
