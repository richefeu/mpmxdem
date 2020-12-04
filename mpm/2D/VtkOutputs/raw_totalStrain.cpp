#include "raw_totalStrain.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, raw_totalStrain> registrar("raw_totalStrain");

void raw_totalStrain::save(std::ostream& os) {
  os << std::endl << "TENSORS raw_totalStrain float" << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << box->MP[i].strain.xx << " " << box->MP[i].strain.xy << " 0" << std::endl;
    os << box->MP[i].strain.yx << " " << box->MP[i].strain.yy << " 0" << std::endl;
    os << "0 0 0" << std::endl;
  }
}