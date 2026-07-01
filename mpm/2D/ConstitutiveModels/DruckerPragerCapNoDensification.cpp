#include "DruckerPragerCapNoDensification.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string DruckerPragerCapNoDensification::getRegistrationName() { return std::string("DruckerPragerCapNoDensification"); }

// ==================================================================================
//  2D version of Drucker-Prager/Cap model (elasto-plastic with hardening)
//   - plane strain
//   - Density dependent yield surface to account for hardening
// ==================================================================================

DruckerPragerCapNoDensification::DruckerPragerCapNoDensification(double young, double poisson, double frictionAngle, double cohesion, double hydroConsolStress,
 double capEccentricity) : E(young), Nu(poisson), Beta(frictionAngle), d(cohesion), Pb(hydroConsolStress), R(capEccentricity) {
  tanBeta = tan(Beta); 
  Pa = (Pb - R*d)/(1+R*tanBeta);
}

void DruckerPragerCapNoDensification::read(std::istream& is) {
  is >> E >> Nu >> Beta >> d >> Pb >> R;
  tanBeta = tan(Beta); 
}

void DruckerPragerCapNoDensification::write(std::ostream& os) {
  os << E << ' ' << Nu << ' ' << Beta << ' ' << d << ' ' << Pb << ' ' << R << '\n';
}

double DruckerPragerCapNoDensification::getYoung() { return E; }

double DruckerPragerCapNoDensification::getPoisson() { return Nu; }

double DruckerPragerCapNoDensification::getYield(double P, double Q) {
  if (P < Pa) {
    return Q - P*tanBeta - d ;
  } 
  else {
    return sqrt((P-Pa)*(P-Pa) + R*R*Q*Q) - R*(d+Pa*tanBeta) ;
  }
}

void DruckerPragerCapNoDensification::updateStrainAndStress(MPMbox& MPM, size_t p) {
  // Get pointer to the first of the nodes
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Compute a strain increment (during dt) from the node-velocities
  // vec2r vn;
  mat4r dstrain;
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
  double a = 1.0 - Nu;
  double b = (1.0 - 2.0 * Nu);
  double f = E / ((1.0 + Nu) * b);
  double De11 = f * a;
  double De12 = f * Nu;
  double De22 = De11;
  double De33 = f * b;

  // Trial stress
  MPM.MP[p].stress.xx += De11 * dstrain.xx + De12 * dstrain.yy;
  MPM.MP[p].stress.yy += De12 * dstrain.xx + De22 * dstrain.yy;
  MPM.MP[p].stress.xy += De33 * dstrain.xy/2;
  MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;

  // q = sqrt(3/2*s:s) = ((sigxx - sigyy)/2)^2 + sigxy^2 
  double diff_xx_yy = MPM.MP[p].stress.xx - MPM.MP[p].stress.yy;
  double Q = sqrt(3*(MPM.MP[p].stress.xy * MPM.MP[p].stress.xy + 0.25 * diff_xx_yy * diff_xx_yy));
  
  double P = -0.5*(MPM.MP[p].stress.xx + MPM.MP[p].stress.yy);
  double yieldF = getYield(P,Q);

  mat4r deltaPlasticStrain;
  if (yieldF > 0.0) {

    if (MPM.MP[p].plastic == false) MPM.MP[p].plastic = true;
		
      int iter = 0;
      while (iter < 10 && yieldF > 1e-10) {  

        iter++;
        double gradfxx,gradfyy,gradfxy;

        if (P < Pa) {
          double inv_Q = 1/Q;  // = 1/((sigxx-sigyy/2)² + sigxy²)

          gradfxx = 0.5*tanBeta + 1.5 * diff_xx_yy * inv_Q;
          gradfyy = 0.5*tanBeta - 1.5 * diff_xx_yy * inv_Q;
          gradfxy = 3 * MPM.MP[p].stress.xy * inv_Q;
        }
        
        else {
          double den = yieldF + R*(d+Pa*tanBeta); // = square root part of the Cap surface function (denominator in function derivative)
          double ThreeRSquared = 3*R*R;

          gradfxx = (-P + Pa + ThreeRSquared*diff_xx_yy) /(2*den);
          gradfyy = (-P + Pa - ThreeRSquared*diff_xx_yy) /(2*den);
          gradfxy = ThreeRSquared*MPM.MP[p].stress.xy / den;
        }

        double bottom_lambda = gradfxx * (De11 * gradfxx + De12 * gradfyy) +
                               gradfyy * (De12 * gradfxx + De22 * gradfyy) + gradfxy * De33 * gradfxy;
        double lambda = yieldF / bottom_lambda;

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
        Q = sqrt(3*(MPM.MP[p].stress.xy * MPM.MP[p].stress.xy + 0.25 * diff_xx_yy * diff_xx_yy));
        P = -0.5*(MPM.MP[p].stress.xx + MPM.MP[p].stress.yy);
        yieldF = getYield(P,Q);
        
      }// end iterations
    }
  // std::cout<<"DruckerPragerCapNoDensification : MP n°"<<p<<", P : " << P << ", Q : " << Q << ", yieldF : "<<yieldF<< std::endl;
}

void DruckerPragerCapNoDensification::init(MaterialPoint& MP) { MP.isDoubleScale = false; }

std::vector<double> DruckerPragerCapNoDensification::getOtherParams(size_t p) {
  p++;
  std::vector<double> params;
  params.push_back(Pb);
  params.push_back(R);
  params.push_back(Beta);
  params.push_back(d);
  return params ;
}