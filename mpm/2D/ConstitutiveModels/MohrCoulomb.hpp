#ifndef MOHRCOULOMB_HPP
#define MOHRCOULOMB_HPP

#include "ConstitutiveModel.hpp"

struct MohrCoulomb : public ConstitutiveModel {
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

  MohrCoulomb(double young = 200.0e6, double poisson = 0.2, double frictionAngle = 0.5, double cohesion = 0.0,
              double dilatancyAngle = 0.3);
  std::string getRegistrationName();
  void read(std::istream& is);
  void write(std::ostream& os);
  double getYoung();
  double getPoisson();
  void updateStrainAndStress(MPMbox& MPM, size_t p);
};

#endif /* end of include guard: MOHRCOULOMB5_HPP */
