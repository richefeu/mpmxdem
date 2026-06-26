#include "raw_totalStress.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, raw_totalStress> registrar("raw_totalStress");

void raw_totalStress::save(std::ostream& os) {
  os << std::endl << "TENSORS raw_totalStress float" << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << box->MP[i].stress.xx << " " << box->MP[i].stress.xy << " 0" << std::endl;
    os << box->MP[i].stress.yx << " " << box->MP[i].stress.yy << " 0" << std::endl;
    os << "0 0 0" << std::endl;
  }
}