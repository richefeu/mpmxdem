#ifndef SINFONIETTA_CLASSICA_HPP
#define SINFONIETTA_CLASSICA_HPP

/**
 * @file SinfoniettaClassica.hpp
 * @brief Header file for the SinfoniettaClassica class.
 *
 * This file contains the declaration of the SinfoniettaClassica class, which is
 * a constitutive model for a material point.
 *
 * The SinfoniettaClassica model is a modified version of the SinfoniettaCrunch
 * model, which is used for the crushing of granular materials.
 *
 * The SinfoniettaClassica model is a simple model that can be used to model
 * the behavior of granular materials under a wide range of conditions.
 *
 * The class relies on various components such as the Young's modulus, Poisson's
 * ratio, and the rigidity of the material point.
 *
 * The file also defines constants and includes necessary headers for the
 * implementation of SinfoniettaClassica.
 */

#include "ConstitutiveModel.hpp"

#include "Rigidity.hpp"

// Ce sera renomer SinfoniettaCrunch
struct SinfoniettaClassica : public ConstitutiveModel {
  double Young;
  double Poisson;
  Rigidity C;

  // See page 49 of Quentin's PhD
  double beta;    // non-associativity characterisation (beta = 3 => associated)
  double beta_p;  // plastic compliance (1/H)
  double kappa;   // shear hardening parameter (often 0)
  double varphi;  // friction angle (characteristic state)
  double pc0;     // pre-consolidation pressure

  // double phi_star_0;
  // double Epv0; // début (approx) du plateau à phi_star_0
  // double l0;
  // double b_inf{1.0};

  SinfoniettaClassica(double young = 200.0e6, double poisson = 0.2);
  std::string getRegistrationName();
  void read(std::istream& is);
  void write(std::ostream& os);
  void init(MaterialPoint & MP);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  double getYoung();
  double getPoisson();

 private:
  double func_f(mat9r Sigma, double q);
  double func_h(mat9r Sigma, double q);
  mat9r func_df_ds(mat9r Sigma, double q);
  mat9r func_dg_ds(mat9r Sigma, double q);
  double func_df_dq(mat9r Sigma, double q);

  const double epsilon_pressure{1e-13};
  const double yield_tol{1e-8};
  double z;  // depends on varphi
};

#endif /* end of include guard: SINFONIETTA_CLASSICA_HPP */
