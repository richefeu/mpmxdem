#include "ConstitutiveModel.hpp"

ConstitutiveModel::~ConstitutiveModel () { }

double ConstitutiveModel::getYoung() {
    std::cout<<"getYoung() not defined for the specified constitutive model"<<std::endl;
    return 0;}
