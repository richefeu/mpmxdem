#ifndef CONSTITUTIVEMODEL_HPP_A78553DE
#define CONSTITUTIVEMODEL_HPP_A78553DE

#include <iostream>
#include <string>

//#include <Core/MPMbox.hpp>
struct MPMbox;
struct MaterialPoint;

struct ConstitutiveModel {
  std::string initFileName;
  //ConstitutiveModel() : initFileName("none") { }
  virtual ~ConstitutiveModel();
  virtual void read(std::istream& is) = 0;
  virtual void updateStrainAndStress(MPMbox& MPM, size_t p) = 0;
  virtual double getYoung();
};

#endif /* end of include guard: CONSTITUTIVEMODEL_HPP_A78553DE */
