#ifndef DRUCKERPRAGERCAP_HPP
#define DRUCKERPRAGERCAP_HPP

/**
 * @file DruckerPragerCap.hpp
 * @brief Defines the DruckerPragerCap constitutive model structure.
 *
 * This header file contains the definition of the DruckerPragerCap structure,
 * which represents a constitutive model used in material point simulations.
 * It inherits from the base class ConstitutiveModel and provides additional
 * parameters such as friction angle and cohesion.
 *
 * The DruckerPragerCap model is used to simulate the behavior of materials
 * that exhibit both plastic and elastic deformation characteristics based
 * on the Drucker-Prager failure criterion.
 *
 * The file declares the private and public members of the DruckerPragerCap
 * structure, including methods for updating strain and stress.
 */

#include "ConstitutiveModel.hpp"
#include <vector>
#include <mat4.hpp>

struct DruckerPragerCap : public ConstitutiveModel {
private:
  double tanBeta;
  // Parameters for the evolution of the yield surface,
  // all values are taken from the results of B.Spanu et al. from UO2 powders post-processed with Cauchy Stress

  // 80 rpm grinding speed
  // double d1 = 6.14e-3;
  // double d2 = 9.68;
  // double Beta = 1.221; // Internal friction angle (70°)
  // double R1 = 0.35;
  // double R2 = 0.42;
  // double Pb1 = 46.876e6;
  // double Pb2 = 10.42;
  // double E1 = 0.07e6;
  // double E2 = 21.01;
  // double Nu1 = -0.37;
  // double Nu2 = 0.45;

  // 60 rpm grinding speed
  double d1  = 5.35e-4;
  double d2  = 13.25;
  double R1  = 0.41;
  double R2  = 0.40;
  double Pb1 = 57.159e6;
  double Pb2 = 11.10;
  double E1  = 0.36e6;
  double E2  = 21.01;
  double Nu1 = -0.23;
  double Nu2 = 0.38;

  double RhoTh = 10.97e3; // Theoretical density of the material in kg/m3 (were it fully solid, not ground)

  double E0;           // Initial value for Young's Modulus
  double Nu0;          // Initial for the Poisson Coefficient
  double Beta = 1.221; // Internal friction angle (70°)
  double d0;           // Initial value for the cohesive strength
  double Pb0;          // Initial value for the maximal Hydrostatic consolidation stress Pb = R(d+Pa*tan(Beta))
  double Pa0;          // Initial value for the hydrostatic stress delimiting Drucker-Prager and Cap sections
  double R0;           // Initial value for the Cap Eccentricity
  double RD0 = 0.3;    // Initial value for the relative density

  double P, Q, yieldF, diff_xx_yy;
  double De11, De12, De22, De33;
  mat4r dstrain;

public:
  // Parameters of the yield surface
  std::vector<double> E;  // Young's Moduli vector
  std::vector<double> Nu; // Poisson Coefficients vector
  std::vector<double> d;  // Cohesive strengths vector
  std::vector<double> Pb; // Maximal Hydrostatic consolidation stresses vector Pb = R(d+Pa*tan(Beta))
  std::vector<double> Pa; // Hydrostatic stresses delimiting Drucker-Prager and Cap sections vector
  std::vector<double> R;  // Cap Eccentricities vector
  std::vector<double> RD; // Relative Densities vector

  // Misc
  int nstep;

  // Default values are calculated with the associated formulas for RD = RD0 = 0.40
  DruckerPragerCap(double young = 46.9e6, double poisson = 0.3, double frictionAngle = 1.221, double cohesion = 2.95e-1,
                   double hydroConsolStress = 3.345e3, double capEccentricity = 0.56);

  std::string getRegistrationName();
  void read(std::istream &is);
  void write(std::ostream &os);
  double getYoung();
  double getPoisson();
  double getYield(double P, double Q, size_t p);
  void updateStrainAndStress(MPMbox &MPM, size_t p);
  void init(MaterialPoint &MP);
  std::vector<double> getOtherParams(size_t p);
};

#endif /* end of include guard: DRUCKERPRAGER5_HPP */
