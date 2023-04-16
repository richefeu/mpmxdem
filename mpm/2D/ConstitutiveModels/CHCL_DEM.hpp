#ifndef CHCL_DEM_HPP
#define CHCL_DEM_HPP

// This is Computationally Homogenised Constitutive Law (CHCL) with DEM

#include "ConstitutiveModel.hpp"
#include "PBC3D.hpp"

struct CHCL_DEM : public ConstitutiveModel {
  CHCL_DEM();
  std::string fileName;  // file name of the initial configuration
	
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

#endif /* end of include guard: CHCL_DEM_HPP */
