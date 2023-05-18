#ifndef SINFONIETTA_CLASSICA_HPP
#define SINFONIETTA_CLASSICA_HPP

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
