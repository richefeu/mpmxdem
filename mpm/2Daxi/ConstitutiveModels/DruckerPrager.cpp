#include "DruckerPrager.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string DruckerPrager::getRegistrationName() { return std::string("DruckerPrager"); }

// ==================================================================================
//  2D version of Drucker-Prager model (elasto-plastic without hardening)
//   - plane strain
//   - take care of the apex-area
//   - DO NOT apply correction on the displacement in case of return to the apex-area
// ==================================================================================

DruckerPrager::DruckerPrager(double young, double poisson, double frictionAngle, double cohesion)
    : Young(young), Poisson(poisson), FrictionAngle(frictionAngle), Cohesion(cohesion) {
  sinFrictionAngle = sin(FrictionAngle);
  cosFrictionAngle = cos(FrictionAngle);
  inv_sqrt3 = 1/sqrt(3);
  alpha = 2*sinFrictionAngle/(sqrt(3)*(3-sinFrictionAngle));
	k = 6*Cohesion*cosFrictionAngle/(sqrt(3)*(3-sinFrictionAngle));
}

void DruckerPrager::read(std::istream& is) {
  is >> Young >> Poisson >> FrictionAngle >> Cohesion;
  sinFrictionAngle = sin(FrictionAngle);
  cosFrictionAngle = cos(FrictionAngle);
  alpha = 2*sinFrictionAngle/(sqrt(3)*(3-sinFrictionAngle));
	k = 6*Cohesion*cosFrictionAngle/(sqrt(3)*(3-sinFrictionAngle));
  // std::cout << "alpha " << alpha <<" k " << k << std::endl;
}

void DruckerPrager::write(std::ostream& os) {
  os << Young << ' ' << Poisson << ' ' << FrictionAngle << ' ' << Cohesion << '\n';
}

double DruckerPrager::getYoung() { return Young; }

double DruckerPrager::getPoisson() { return Poisson; }

double DruckerPrager::getYield(double P, double Q) {
  return 2*alpha*P + inv_sqrt3*Q - k;
}

void DruckerPrager::updateStrainAndStress(MPMbox& MPM, size_t p) {
  // Get pointer to the first of the nodes
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Compute a strain increment (during dt) from the node-velocities
  // vec2r vn;
  mat4r dstrain;
  double out_of_plane_dstrain;
  for (size_t r = 0; r < element::nbNodes; r++) {
    dstrain.xx += (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r][0]) * MPM.dt;
    dstrain.xy +=
        0.5 * (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r][1] + MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r][0]) * MPM.dt;
    dstrain.yy += (MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r][1]) * MPM.dt;
    out_of_plane_dstrain += MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r][2]; 
  }
  dstrain.yx = dstrain.xy;

  MPM.MP[p].strain += dstrain;
  MPM.MP[p].outOfPlaneStrain += out_of_plane_dstrain;
  MPM.MP[p].deltaStrain = dstrain;
  MPM.MP[p].outOfPlaneDeltaStrain = out_of_plane_dstrain;

  //      |De11 De12 0   |       |a        Poisson  0  |        with a = 1 - Poisson
  // De = |De12 De22 0   | = f * |Poisson  a        0  |             b = 1 - 2Poisson
  //      |0    0    De33|       |0        0        b/2|         and f = Young / (1 + 2Poisson)
  double a = 1.0 - Poisson;
  double b = (1.0 - 2.0 * Poisson);
  double f = Young / ((1.0 + Poisson) * b);
  double De11 = f * a;
  double De12 = f * Poisson;
  double De22 = De11;
  double De33 = f * b;

  // Trial stress
  MPM.MP[p].stress.xx += De11 * dstrain.xx + De12 * dstrain.yy;
  MPM.MP[p].stress.yy += De12 * dstrain.xx + De22 * dstrain.yy;
  MPM.MP[p].stress.xy += 2 * De33 * dstrain.xy;
  MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;

  // q = sqrt(3/2*s:s) = ((sigxx - sigyy)/2)^2 + sigxy^2 
  double diff_xx_yy = MPM.MP[p].stress.xx - MPM.MP[p].stress.yy;
  double Q = sqrt(1.5*(MPM.MP[p].stress.xy * MPM.MP[p].stress.xy + 0.25 * diff_xx_yy * diff_xx_yy));
  
  double P = -0.5*(MPM.MP[p].stress.xx + MPM.MP[p].stress.yy);
  double yieldF = getYield(P,Q);
  // std::cout << "yieldF : " << yieldF << std::endl;

  mat4r deltaPlasticStrain;
  if (yieldF > 0.0) {

    if (MPM.MP[p].plastic == false) MPM.MP[p].plastic = true;
		
    // Check if sigma is outside the 'apex-area'

    double apex = 2*sqrt(3)*alpha * Q + 0.5*k/alpha;

    if (P <= apex) {                          // okay, we can iterate
      int iter = 0;
      while (iter < 50 && yieldF > 1e-10) {  // Actually a single iteration should be okay	HERE WE HAD || INSTEAD
                                             // OF && AND WAS A SOURCE OF ISSUES

        iter++;
        double sqrt3_Q = sqrt(3) / Q;  

        double gradfxx = 2*alpha + diff_xx_yy * sqrt3_Q;
        double gradfyy = 2*alpha - diff_xx_yy * sqrt3_Q;
        double gradfxy = 2 * MPM.MP[p].stress.xy * sqrt3_Q;

        double bottom_lambda = gradfxx * (De11 * gradfxx + De12 * gradfyy) +
                               gradfyy * (De12 * gradfxx + De22 * gradfyy) + gradfxy * De33 * gradfxy;
        double lambda = yieldF / bottom_lambda;

        deltaPlasticStrain.xx = lambda * gradfxx;
        deltaPlasticStrain.yy = lambda * gradfyy;
        deltaPlasticStrain.xy = lambda * gradfxy;
        deltaPlasticStrain.yx = deltaPlasticStrain.xy;

        MPM.MP[p].plasticStrain += deltaPlasticStrain;  // FIXME: INCREMENTED AT EACH ITERATION?

        // Correcting state of stress
        mat4r delta_sigma_corrector;
        delta_sigma_corrector.xx = De11 * deltaPlasticStrain.xx + De12 * deltaPlasticStrain.yy;
        delta_sigma_corrector.yy = De12 * deltaPlasticStrain.xx + De22 * deltaPlasticStrain.yy;
        delta_sigma_corrector.xy = De33 * deltaPlasticStrain.xy;
        delta_sigma_corrector.yx = delta_sigma_corrector.xy;

        // correcting the current state of stress
        MPM.MP[p].stress -= delta_sigma_corrector;

        // saving the plastic stress to a mat4 variable
        MPM.MP[p].stressCorrection += delta_sigma_corrector;

        // New value of the yield function
        diff_xx_yy = MPM.MP[p].stress.xx - MPM.MP[p].stress.yy;
        Q = sqrt(1.5*(MPM.MP[p].stress.xy * MPM.MP[p].stress.xy + 0.25 * diff_xx_yy * diff_xx_yy));
        P = 0.5*(MPM.MP[p].stress.xx + MPM.MP[p].stress.yy);
        yieldF = getYield(P,Q);
      }// end iterations
      
    } else {  // case in the apex-area
      MPM.MP[p].stress.xx = MPM.MP[p].stress.yy = 0.5*k/alpha;
      MPM.MP[p].stress.xy = MPM.MP[p].stress.yx = 0.0;
    }
  }
  // if (p==1) {
  //       if (nstep%100 == 0)
  //       {std::cout<<"Model : " << yieldF<< std::endl;}
  //       nstep+=1;
  //       }
}

void DruckerPrager::init(MaterialPoint& MP) { MP.isDoubleScale = false; }
