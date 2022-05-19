#include "ConstitutiveModel.hpp"

ConstitutiveModel::~ConstitutiveModel() {}

double ConstitutiveModel::getYoung() {
  std::cout << "getYoung() not defined for the specified constitutive model" << std::endl;
  return 0;
}

double ConstitutiveModel::getPoisson() {
  std::cout << "getPoisson() not defined for the specified constitutive model" << std::endl;
  return 0;
}

void ConstitutiveModel::init(MaterialPoint & /*MP*/) { }
