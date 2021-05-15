#ifndef HOOKEELASTICITY_HPP
#define HOOKEELASTICITY_HPP

#include "ConstitutiveModel.hpp"

struct HookeElasticity : public ConstitutiveModel {
  double Young;
  double Poisson;

  HookeElasticity(double young = 200.0e6, double poisson = 0.2);
  std::string getRegistrationName();
  void read(std::istream& is);
  void write(std::ostream& os);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  double getYoung();
};

#endif /* end of include guard: HOOKEELASTICITY_HPP */
