#ifndef VONMISESELASTOPLASTICITY_HPP_B1E4F642
#define VONMISESELASTOPLASTICITY_HPP_B1E4F642

#include "ConstitutiveModel.hpp"

struct VonMisesElastoPlasticity : public ConstitutiveModel {
  double Young;
  double Poisson;
  double PlasticYieldStress;

  VonMisesElastoPlasticity(double young = 200.0e6, double poisson = 0.2, double plasticYieldStress = 100.0e3);

  void read(std::istream& is);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
};

#endif /* end of include guard: VONMISESELASTOPLASTICITY_HPP_B1E4F642 */
