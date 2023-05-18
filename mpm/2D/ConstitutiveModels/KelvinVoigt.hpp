#ifndef KELVIN_VOIGT_HPP
#define KELVIN_VOIGT_HPP

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
  double Young;
  double Poisson;
  Rigidity C;
  double eta;
};

#endif /* end of include guard: KELVIN_VOIGT_HPP */
