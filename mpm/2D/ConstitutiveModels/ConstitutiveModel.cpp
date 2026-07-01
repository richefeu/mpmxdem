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

void ConstitutiveModel::init([[maybe_unused]] MaterialPoint & MP) { }

std::vector<double> ConstitutiveModel::getOtherParams(size_t p) {
  p++;
  std::cout<< "getOtherParams() not defined for the specified constitutive model"<<std::endl;
}