#ifndef NEOHOOKEAN_HPP_6CF02188
#define NEOHOOKEAN_HPP_6CF02188

#include "ConstitutiveModel.hpp"

struct NeoHookean : public ConstitutiveModel {
  double Young;
  double Poisson;

  NeoHookean(double young = 200.0e6, double poisson = 0.2);

  void read(std::istream& is);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
};

#endif /* end of include guard: NEOHOOKEAN_HPP_6CF02188 */
