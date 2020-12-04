#include "FiniteElasticity.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<ConstitutiveModel, FiniteElasticity> registrar("FiniteElasticity");

FiniteElasticity::FiniteElasticity(double young, double poisson) : Young(young), Poisson(poisson) {}

void FiniteElasticity::read(std::istream& is) { is >> Young >> Poisson; }

void FiniteElasticity::updateStrainAndStress(MPMbox& MPM, size_t p) {
  // Get the total strain increment from node velocities
  vec2r vn;
  mat4 dstrain;

  // the best method so far is the one were using (hookeelasticiy). the others simply dont work

  /*
  //Method 1. Nguyen(mpm in julia[2017] pg 13). Strange results. Strain is always 0 ina free fall (since F is not
  changing) MPM.MP[p].strain.xx += MPM.MP[p].Fincrement.xx - 1; MPM.MP[p].strain.yy += MPM.MP[p].Fincrement.yy - 1;
  MPM.MP[p].strain.xy += MPM.MP[p].Fincrement.xy + MPM.MP[p].Fincrement.yx;
  MPM.MP[p].strain.yx = MPM.MP[p].strain.xy;
  */
  // MPM.MP[p].strain = MPM.MP[p].F * MPM.MP[p].F.transpose();  //Method 2. green tensor (continuuum foam)  //giving
  // weird results

  // Method 3. nguyen (mpm libre) pg 29. Works well
  dstrain = MPM.dt * 0.5 * (MPM.MP[p].velGrad + MPM.MP[p].velGrad.transpose());
  MPM.MP[p].strain += dstrain;

  // MPM.MP[p].strain += dstrain;

  //      |De11 De12 0   |       |a        Poisson  0  |  with a = 1-Poisson
  // De = |De12 De22 0   | = f * |Poisson  a        0  |       b = 1-2Poisson
  //      |0    0    De33|       |0        0        b/2|   and f = Young/(1+2Poisson)
  double a = 1.0 - Poisson;
  double b = (1.0 - 2.0 * Poisson);
  double f = Young / ((1.0 + Poisson) * b);
  double De11 = f * a;
  double De12 = f * Poisson;
  double De22 = De11;
  double De33 = f * 0.5 * b;

  // Elastic stress
  MPM.MP[p].stress.xx = De11 * MPM.MP[p].strain.xx + De12 * MPM.MP[p].strain.yy;
  MPM.MP[p].stress.yy = De12 * MPM.MP[p].strain.xx + De22 * MPM.MP[p].strain.yy;
  MPM.MP[p].stress.xy = 2 * De33 * MPM.MP[p].strain.xy;
  MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;
}
