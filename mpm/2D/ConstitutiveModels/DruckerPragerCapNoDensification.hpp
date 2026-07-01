#ifndef DRUCKERPRAGERCAPNODENSIFICATION_HPP
#define DRUCKERPRAGERCAPNODENSIFICATION_HPP

/**
 * @file DruckerPragerCapNoDensification.hpp
 * @brief Defines the DruckerPragerCapNoDensification constitutive model structure.
 *
 * This header file contains the definition of the DruckerPragerCapNoDensification structure,
 * which represents a constitutive model used in material point simulations.
 * It inherits from the base class ConstitutiveModel and provides additional
 * parameters such as friction angle and cohesion.
 *
 * The DruckerPragerCapNoDensification model is used to simulate the behavior of materials
 * that exhibit both plastic and elastic deformation characteristics based
 * on the Drucker-Prager failure criterion.
 *
 * The file declares the private and public members of the DruckerPragerCapNoDensification
 * structure, including methods for updating strain and stress.
 */

#include "ConstitutiveModel.hpp"
#include <vector>

struct DruckerPragerCapNoDensification : public ConstitutiveModel {
 private:
  double tanBeta;
  // Parameters for the evolution of the yield surface, 
  // all values are taken from the results of B.Spanu et al. from UO2 powders ground at 80 rpm (Cauchy stress post-processing)
    double E1 = 0.07e6;
    double E2 = 21.01; 
    double Nu1 = -0.37;
    double Nu2 = 0.45;
    double d1 = 6.14e-3;
    double d2 = 9.68;
    double Pb1 = 46.876e6;
    double Pb2 = 10.42;
    double R1 = 0.35;
    double R2 = 0.42;
    // double Rho; // Mean Density in kg/m3
    double RhoTh = 10.97e3; // Theoretical density of the material in kg/m3 (were it fully solid, not ground)
   
 public:
  // Parameters of the yield surface
  double E;     // Young's Modulus
  double Nu;    // Poisson Coefficient
  double Beta = 1.221; // Internal friction angle (70°)
  double d;     // Cohesive strength
  double Pb;    // Maximal Hydrostatic consolidation stress Pb = R(d+Pa*tan(Beta))
  double Pa;    // Hydrostatic stress delimiting Drucker-Prager and Cap sections
  double R;     // Cap Eccentricity
  // double RD;   // Relative Density
  

  // Misc
  int nstep;
  
  //Default values are calculated with the associated formulas for RD = RD0 = 0.40 
  DruckerPragerCapNoDensification(double young = 46.9e6, double poisson = 0.3, double frictionAngle = 1.221 , double cohesion = 2.95e-1,
   double hydroConsolStress = 3.345e3, double capEccentricity = 0.56); 

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
