#include "DruckerPragerCap.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string DruckerPragerCap::getRegistrationName() {
  return std::string("DruckerPragerCap");
}

// ==================================================================================
//  2D version of Drucker-Prager/Cap model (elasto-plastic with hardening)
//   - Plane strain hypothesis
//   - Density dependent yield surface to account for hardening
// ==================================================================================

DruckerPragerCap::DruckerPragerCap(double young, double poisson, double frictionAngle, double cohesion,
                                   double hydroConsolStress, double capEccentricity)
    : E0(young), Nu0(poisson), Beta(frictionAngle), d0(cohesion), Pb0(hydroConsolStress), R0(capEccentricity) {
  tanBeta = tan(Beta);
  Pa0     = (Pb0 - R0 * d0) / (1 + R0 * tanBeta);
}

void DruckerPragerCap::read(std::istream &is) {
  is >> E0 >> Nu0 >> Beta >> d0 >> Pb0 >> R0;
  tanBeta = tan(Beta);
}

void DruckerPragerCap::write(std::ostream &os) {
  os << E0 << ' ' << Nu0 << ' ' << Beta << ' ' << d0 << ' ' << Pb0 << ' ' << R0 << '\n';
}

double DruckerPragerCap::getYoung() {
  return E0;
}

double DruckerPragerCap::getPoisson() {
  return Nu0;
}

double DruckerPragerCap::getYield(double P, double Q, size_t p) {
  if (P < Pa[p]) { return Q - P * tanBeta - d[p]; }
  return sqrt((P - Pa[p]) * (P - Pa[p]) + R[p] * R[p] * Q * Q) - R[p] * (d[p] + Pa[p] * tanBeta);
}

void DruckerPragerCap::updateStrainAndStress(MPMbox &MPM, size_t p) {
  if (MPM.step == 0) { // initialize DPC material properties vectors
    START_TIMER("initializeDPCMaterialVectors");
    E.push_back(E1 * std::exp(E2 * RD0));
    Nu.push_back(Nu1 * RD0 + Nu2);
    Pb.push_back(Pb1 * std::exp(Pb2 * std::log(RD0)));
    R.push_back(R1 * RD0 + R2);
    d.push_back(d1 * std::exp(d2 * RD0));
    Pa.push_back((Pb[p] - R[p] * d[p]) / (1 + R[p] * tanBeta));
    RD.push_back(RD0);
  }
  // Get pointer to the first of the nodes
  size_t *I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  {
    START_TIMER("computeStrainIncrement");
    // Compute a strain increment (during dt) from the node-velocities
    // vec2r vn;
    dstrain.reset();
    for (size_t r = 0; r < element::nbNodes; r++) {
      dstrain.xx += (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].x) * MPM.dt;
      dstrain.xy +=
          0.5 * (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].y + MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].x) * MPM.dt;
      dstrain.yy += (MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].y) * MPM.dt;
    }
    dstrain.yx = dstrain.xy;

    MPM.MP[p].deltaStrain = dstrain; // store it for processing

    MPM.MP[p].strain += dstrain; // increment the strain

    //      |De11 De12 0   |       |a   Nu  0| with a = 1 - Nu
    // De = |De12 De22 0   | = f * |Nu  a   0|      b = 1 - 2Nu
    //      |0    0    De33|       |0   0   b|      and f = E/(1+Nu)(1 - 2Nu)
    double a = 1.0 - Nu[p];
    double b = (1.0 - 2.0 * Nu[p]);
    double f = E[p] / ((1.0 + Nu[p]) * b);
    De11     = f * a;
    De12     = f * Nu[p];
    De22     = De11;
    De33     = f * b;
  }
  {
    START_TIMER("computeTrialStress");
    // Trial stress
    MPM.MP[p].stress.xx += De11 * dstrain.xx + De12 * dstrain.yy;
    MPM.MP[p].stress.yy += De12 * dstrain.xx + De22 * dstrain.yy;
    MPM.MP[p].stress.xy += De33 * dstrain.xy / 2;
    MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;

    // q = sqrt(3/2*s:s) = ((sigxx - sigyy)/2)^2 + sigxy^2
    diff_xx_yy = MPM.MP[p].stress.xx - MPM.MP[p].stress.yy;
    Q          = sqrt(3 * (MPM.MP[p].stress.xy * MPM.MP[p].stress.xy + 0.25 * diff_xx_yy * diff_xx_yy));
    P          = -0.5 * (MPM.MP[p].stress.xx + MPM.MP[p].stress.yy);
    yieldF     = getYield(P, Q, p);
  }
  mat4r deltaPlasticStrain;
  if (yieldF > 0.0) {

    if (MPM.MP[p].plastic == false) MPM.MP[p].plastic = true;

    // update DPC material properties of the MP (expressions are derived from experimental regressions from B. Spanu
    // et al.)
    {
      START_TIMER("updateDPCParameters");
      RD[p] = MPM.MP[p].density / RhoTh;
      E[p]  = E1 * std::exp(E2 * RD[p]);
      Nu[p] = Nu1 * RD[p] + Nu2;
      Pb[p] = Pb1 * std::exp(Pb2 * std::log(RD[p]));
      R[p]  = R1 * RD[p] + R2;
      d[p]  = d1 * std::exp(d2 * RD[p]);
      Pa[p] = (Pb[p] - R[p] * d[p]) / (1 + R[p] * tanBeta);
    }
    {
      START_TIMER("computePlasticCorrection");
      int iter = 0;
      while (iter < 10 && yieldF > 1e-10) {

        iter++;
        double gradfxx, gradfyy, gradfxy;

        if (P < Pa[p]) {
          double inv_Q = 1 / Q; // = 1/((sigxx-sigyy/2)² + sigxy²)

          gradfxx = 0.5 * tanBeta + 1.5 * diff_xx_yy * inv_Q;
          gradfyy = 0.5 * tanBeta - 1.5 * diff_xx_yy * inv_Q;
          gradfxy = 3 * MPM.MP[p].stress.xy * inv_Q;
        }

        else {
          double den = yieldF + R[p] * (d[p] +  Pa[p] * tanBeta); // = square root part of the Cap surface function (denominator in function derivative)
          double ThreeRSquared = 3 * R[p] * R[p];

          gradfxx = (-P + Pa[p] + ThreeRSquared * diff_xx_yy) / (2 * den);
          gradfyy = (-P + Pa[p] - ThreeRSquared * diff_xx_yy) / (2 * den);
          gradfxy = ThreeRSquared * MPM.MP[p].stress.xy / den;
        }

        double bottom_lambda = gradfxx * (De11 * gradfxx + De12 * gradfyy) +
                                gradfyy * (De12 * gradfxx + De22 * gradfyy) + gradfxy * De33 * gradfxy;
        double lambda        = yieldF / bottom_lambda;

        deltaPlasticStrain.xx = lambda * gradfxx;
        deltaPlasticStrain.yy = lambda * gradfyy;
        deltaPlasticStrain.xy = lambda * gradfxy;
        deltaPlasticStrain.yx = deltaPlasticStrain.xy;

        MPM.MP[p].plasticStrain += deltaPlasticStrain;

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
        Q          = sqrt(3 * (MPM.MP[p].stress.xy * MPM.MP[p].stress.xy + 0.25 * diff_xx_yy * diff_xx_yy));
        P          = -0.5 * (MPM.MP[p].stress.xx + MPM.MP[p].stress.yy);
        yieldF     = getYield(P, Q, p);

      } // end iterations
    }
  }
}


void DruckerPragerCap::init(MaterialPoint &MP) {
  MP.isDoubleScale = false;
}

std::vector<double> DruckerPragerCap::getOtherParams(size_t p) {
  std::vector<double> params;
  params.push_back(Pb[p]);
  params.push_back(R[p]);
  params.push_back(Beta);
  params.push_back(d[p]);
  params.push_back(E[p]);
  params.push_back(Nu[p]);
  params.push_back(RD[p]);
  return params;
}
