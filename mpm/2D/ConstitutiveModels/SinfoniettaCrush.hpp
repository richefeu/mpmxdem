#ifndef SINFONIETTA_CRUSH_HPP
#define SINFONIETTA_CRUSH_HPP

#include "ConstitutiveModel.hpp"

#include "Rigidity.hpp"
#include "transitFunc.hpp"

struct SinfoniettaCrush : public ConstitutiveModel {
  double Young;
  double Poisson;
  Rigidity C;

  // See page 49 of Quentin's PhD
  double beta{0.0};    // non-associativity characterisation (beta = 3 => associated)
  double beta_p{0.0};  // plastic compliance (1/H)
  double kappa{0.0};   // shear hardening parameter (often 0)
  double varphi{0.0};  // friction angle (characteristic state)
  double pc0{0.0};     // pre-consolidation pressure

  double phiStar0{0.0};   // poro libérable initiale
  linearToPlateau bfunc;  //
  double ginf{0.0};
  double epv0{0.0};
  double l0{0.0};

  SinfoniettaCrush(double young = 200.0e6, double poisson = 0.2);
  std::string getRegistrationName();
  void read(std::istream& is);
  void write(std::ostream& os);
  void init(MaterialPoint& MP);
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  double getYoung();
  double getPoisson();

 private:
  double func_f(mat9r Sigma, double q);
  double func_h(mat9r Sigma, double q, mat9r Ep);
  mat9r func_df_ds(mat9r Sigma, double q);
  mat9r func_dg_ds(mat9r Sigma, double q);
  double func_df_dq(mat9r Sigma, double q);

  const double epsilon_pressure{1e-13};
  const double yield_tol{1e-8};
  double z{0.0};  // depends on varphi
};

#endif /* end of include guard: SINFONIETTA_CRUSH_HPP */
