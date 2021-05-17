#ifndef HNLDEM_HPP
#define HNLDEM_HPP

// This is Homogenised Numerical Law (HNL) with DEM

#include "ConstitutiveModel.hpp"
#include "PBC3D.hpp"

struct hnlDEM : public ConstitutiveModel {
  hnlDEM();
  //std::string fn;
  std::string fileName;

  virtual std::string getRegistrationName();
  void updateStrainAndStress(MPMbox& MPM, size_t p);
  double getYoung();
  void read(std::istream& is);
  void write(std::ostream& os);

  mat9r F;
  mat9r h;
};

#endif /* end of include guard: HNLDEM_HPP */
