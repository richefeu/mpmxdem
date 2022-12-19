#include "HookeElasticity.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

//#include "factory.hpp"
//static Registrar<ConstitutiveModel, HookeElasticity> registrar("HookeElasticity");
std::string HookeElasticity::getRegistrationName() { return std::string("HookeElasticity"); }

HookeElasticity::HookeElasticity(double young, double poisson) : Young(young), Poisson(poisson) {}

void HookeElasticity::read(std::istream& is) { is >> Young >> Poisson; }
void HookeElasticity::write(std::ostream& os) { os << Young << ' ' << Poisson << '\n'; }

void HookeElasticity::updateStrainAndStress(MPMbox& MPM, size_t p) {
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Get the total strain increment from node velocities
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

  MPM.MP[p].strain += dstrain;
  MPM.MP[p].deltaStrain = dstrain;

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
  MPM.MP[p].stress.xx += De11 * dstrain.xx + De12 * dstrain.yy;
  MPM.MP[p].stress.yy += De12 * dstrain.xx + De22 * dstrain.yy;
  MPM.MP[p].stress.xy += 2 * De33 * dstrain.xy;
  MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;
}

double HookeElasticity::getYoung() { return Young; }

double HookeElasticity::getPoisson() { return Poisson; }
