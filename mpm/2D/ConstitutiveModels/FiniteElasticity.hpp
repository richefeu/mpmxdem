#ifndef FINITEELASTICITY_HPP_6CF02188
#define FINITEELASTICITY_HPP_6CF02188

#include "ConstitutiveModel.hpp"

struct FiniteElasticity : public ConstitutiveModel {
  double Young;
  double Poisson;

  FiniteElasticity(double young = 200.0e6, double poisson = 0.2);

  void read(std::istream& is);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
};

#endif /* end of include guard: FINITEELASTICITY_HPP_6CF02188 */
