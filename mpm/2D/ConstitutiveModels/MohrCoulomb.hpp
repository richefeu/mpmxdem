#ifndef MOHRCOULOMB_HPP
#define MOHRCOULOMB_HPP

/**
 * @file MohrCoulomb.hpp
 * @brief Defines the MohrCoulomb constitutive model structure.
 *
 * This header file contains the definition of the MohrCoulomb structure,
 * which represents a constitutive model used in material point simulations.
 * It inherits from the base class ConstitutiveModel and provides additional
 * parameters such as friction angle, cohesion, and dilatancy angle.
 *
 * The MohrCoulomb model is used to simulate the behavior of materials
 * that exhibit both plastic and elastic deformation characteristics based
 * on the Mohr-Coulomb failure criterion.
 *
 * The file declares the private and public members of the MohrCoulomb
 * structure, including methods for updating strain and stress.
 */

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
  void init(MaterialPoint & MP);
};

#endif /* end of include guard: MOHRCOULOMB5_HPP */
