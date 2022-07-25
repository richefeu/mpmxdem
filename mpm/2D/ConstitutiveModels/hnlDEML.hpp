#ifndef HNLDEML_HPP
#define HNLDEML_HPP

// This is Homogenised Numerical Law (HNL) with DEM for loose structures

#include "ConstitutiveModel.hpp"
#include "PBC3D.hpp"

struct hnlDEML : public ConstitutiveModel {
  hnlDEML();
  std::string fileName;  // file name of the initial configuration
  
  double timeBondReactivation;
  double bondingFactor;
  // Different MP can use the same initial conf,
  // but each MP holds its own simulation

  virtual std::string getRegistrationName();
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  double getYoung();
  double getPoisson();
  void read(std::istream& is);
  void write(std::ostream& os);
  void init(MaterialPoint & MP);
};

#endif /* end of include guard: HNLDEML_HPP */
