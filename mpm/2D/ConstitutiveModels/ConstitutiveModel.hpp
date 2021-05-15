#ifndef CONSTITUTIVEMODEL_HPP
#define CONSTITUTIVEMODEL_HPP

#include <iostream>
#include <string>

struct MPMbox;
struct MaterialPoint;

struct ConstitutiveModel {
  virtual ~ConstitutiveModel();
  virtual std::string getRegistrationName() = 0;
  virtual void read(std::istream& is) = 0;
  virtual void write(std::ostream& os) = 0;
  virtual void updateStrainAndStress(MPMbox& MPM, size_t p) = 0;
  virtual double getYoung();
};

#endif /* end of include guard: CONSTITUTIVEMODEL_HPP */
