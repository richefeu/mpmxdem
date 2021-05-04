#include "ConstitutiveModel.hpp"
#include <PBC3D.hpp>


struct hnlDEM : public ConstitutiveModel {
  hnlDEM();
  std::string fn;
  void updateStrainAndStress(MPMbox& MPM,size_t p);
  double getYoung();
  void read(std::istream& is);

  mat9r F; mat9r h;
};

