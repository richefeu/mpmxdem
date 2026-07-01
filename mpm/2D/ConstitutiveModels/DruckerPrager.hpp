#ifndef DRUCKERPRAGER_HPP
#define DRUCKERPRAGER_HPP

/**
 * @file DruckerPrager.hpp
 * @brief Defines the DruckerPrager constitutive model structure.
 *
 * This header file contains the definition of the DruckerPrager structure,
 * which represents a constitutive model used in material point simulations.
 * It inherits from the base class ConstitutiveModel and provides additional
 * parameters such as friction angle and cohesion.
 *
 * The DruckerPrager model is used to simulate the behavior of materials
 * that exhibit both plastic and elastic deformation characteristics based
 * on the Drucker-Prager failure criterion.
 *
 * The file declares the private and public members of the DruckerPrager
 * structure, including methods for updating strain and stress.
 */

#include "ConstitutiveModel.hpp"

struct DruckerPrager : public ConstitutiveModel {
 private:
  double sinFrictionAngle;
  double cosFrictionAngle;
  double alpha;
  double k;
  double inv_sqrt3;

 public:
  double Young;
  double Poisson;
  double FrictionAngle;
  double Cohesion;
  int nstep;

  DruckerPrager(double young = 1.0e5, double poisson = 0.3, double frictionAngle = 0.523598 /*= pi/6 */,
               double cohesion = 1.0);
  std::string getRegistrationName();
  void read(std::istream& is);
  void write(std::ostream& os);
  double getYoung();
  double getPoisson();
  double getYield(double P, double Q);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  void init(MaterialPoint & MP);
  virtual std::vector<double> getOtherParams(size_t p);
};

#endif /* end of include guard: DRUCKERPRAGER5_HPP */
