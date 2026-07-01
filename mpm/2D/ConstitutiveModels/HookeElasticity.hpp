#pragma once

// 
// Implements the Hooke's law for small deformations: sigma = C * epsilon
// 

#include "ConstitutiveModel.hpp"

#include "Rigidity.hpp"
#include <vector>

struct HookeElasticity : public ConstitutiveModel {

  HookeElasticity(double young = 200.0e6, double poisson = 0.2);
  std::string getRegistrationName();
  void read(std::istream& is);
  void write(std::ostream& os);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  double getYoung();
  double getPoisson();
  std::vector<double> getOtherParams(size_t p);

private:
  double Young;
  double Poisson;
  Rigidity C;
};
