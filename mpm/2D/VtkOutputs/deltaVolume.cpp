#include "deltaVolume.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, deltaVolume> registrar("deltaVolume");

void deltaVolume::save(std::ostream& os) {
  os << std::endl << "SCALARS deltaVolume float 1" << std::endl;
  os << "LOOKUP_TABLE default" << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << (box->MP[i].strain.xx + box->MP[i].strain.yy) * box->MP[i].vol << std::endl;
  }
}