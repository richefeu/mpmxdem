#ifndef HNLDEM_HPP
#define HNLDEM_HPP

// This is Homogenised Numerical Law (HNL) with DEM

#include "ConstitutiveModel.hpp"
#include "PBC3D.hpp"

struct hnlDEM : public ConstitutiveModel {
  hnlDEM();
  std::string fileName;  // file name of the initial configuration
  double etaDamping;
  double timeBonds;
  double distBonds;
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

#endif /* end of include guard: HNLDEM_HPP */
