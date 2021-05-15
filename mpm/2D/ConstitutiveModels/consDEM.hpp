#ifndef CONSDEM_HPP
#define CONSDEM_HPP

#include "ConstitutiveModel.hpp"
#include "PBC3D.hpp"

struct consDEM : public ConstitutiveModel {
  char const * dname;
  std::string dnastr;
  
  consDEM(const char * fileName=nullptr);
  std::string getRegistrationName();
  void updateStrainAndStress(MPMbox& MPM,size_t p);
  double getYoung();
  void read(std::istream& is);
  void write(std::ostream& os);

  mat9r F; mat9r h;
};

#endif /* end of include guard: CONSDEM_HPP */
