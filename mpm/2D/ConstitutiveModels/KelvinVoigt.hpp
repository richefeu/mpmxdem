#pragma once

#include "ConstitutiveModel.hpp"

#include "Rigidity.hpp"

struct KelvinVoigt : public ConstitutiveModel {

  KelvinVoigt(double young = 1e6, double poisson = 0.2);
  std::string getRegistrationName();
  void read(std::istream& is);
  void write(std::ostream& os);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  double getYoung();
  double getPoisson();

private:
  double Young{0.0};
  double Poisson{0.0};
  Rigidity C;
  double eta{0.0};
};
