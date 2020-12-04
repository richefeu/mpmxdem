#include "NeoHookean.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<ConstitutiveModel, NeoHookean> registrar("NeoHookean");

NeoHookean::NeoHookean(double young, double poisson) : Young(young), Poisson(poisson) {}

void NeoHookean::read(std::istream& is) { is >> Young >> Poisson; }

void NeoHookean::updateStrainAndStress(MPMbox& MPM, size_t p) {
  double Lame1 = 0.5 * Young / (1.0 + Poisson);
  double Lame2 = Poisson * Young / ((1 + Poisson) * (1.0 - 2.0 * Poisson));
  double J = MPM.MP[p].F.det();

  // Elastic stress
  MPM.MP[p].stress =
      Lame2 * log(J) / J * mat4::unit() + Lame1 / J * (MPM.MP[p].F * MPM.MP[p].F.transpose() - mat4::unit());
}
