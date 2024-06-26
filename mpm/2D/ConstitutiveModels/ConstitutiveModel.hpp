#ifndef CONSTITUTIVEMODEL_HPP
#define CONSTITUTIVEMODEL_HPP

#include <iostream>
#include <string>

class MPMbox;
struct MaterialPoint;

struct ConstitutiveModel {
  std::string key;
	MPMbox *box;

  virtual ~ConstitutiveModel();
  virtual std::string getRegistrationName() = 0;
  virtual void read(std::istream& is) = 0;
  virtual void write(std::ostream& os) = 0;
  virtual void updateStrainAndStress(MPMbox& MPM, size_t p) = 0;
  virtual double getYoung();
  virtual double getPoisson();
  virtual void init(MaterialPoint & MP);
};

#endif /* end of include guard: CONSTITUTIVEMODEL_HPP */
