#ifndef VONMISESELASTOPLASTICITY_HPP
#define VONMISESELASTOPLASTICITY_HPP

#include "ConstitutiveModel.hpp"

struct VonMisesElastoPlasticity : public ConstitutiveModel {
  double Young;
  double Poisson;
  double PlasticYieldStress;

  VonMisesElastoPlasticity(double young = 200.0e6, double poisson = 0.2, double plasticYieldStress = 100.0e3);
  std::string getRegistrationName();
  void read(std::istream& is);
  void write(std::ostream& os);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  double getYoung();
  double getPoisson();
};

#endif /* end of include guard: VONMISESELASTOPLASTICITY_HPP */
