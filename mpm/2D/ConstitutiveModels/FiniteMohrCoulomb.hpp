#ifndef FINITEMOHRCOULOMB_HPP_4F7FFG1E
#define FINITEMOHRCOULOMB_HPP_4F7FFG1E

#include "ConstitutiveModel.hpp"

struct FiniteMohrCoulomb : public ConstitutiveModel {
 private:
  double sinFrictionAngle;
  double cosFrictionAngle;
  double sinDilatancyAngle;

 public:
  double Young;
  double Poisson;
  double FrictionAngle;
  double Cohesion;
  double DilatancyAngle;

  FiniteMohrCoulomb(double young = 200.0e6, double poisson = 0.2, double frictionAngle = 0.5, double cohesion = 0.0,
                    double dilatancyAngle = 0.3);

  void read(std::istream& is);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
};

#endif /* end of include guard: FINITEMOHRCOULOMB5_HPP_4F7FFG1E */
