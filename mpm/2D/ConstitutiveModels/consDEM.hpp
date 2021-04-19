#include "ConstitutiveModel.hpp"
#include <PBC3D.hpp>


struct consDEM : public ConstitutiveModel {
  char const * dname;
  std::string dnastr;
  consDEM(const char * fileName=nullptr);

  void updateStrainAndStress(MPMbox& MPM,size_t p);
  double getYoung();
  void read(std::istream& is);

  mat9r F; mat9r h;
};

