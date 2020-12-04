#include "anything.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, anything> registrar("anything");

void anything::save(std::ostream& os) {
  os << std::endl << "SCALARS anything float 1" << std::endl;
  os << "LOOKUP_TABLE default" << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << box->MP[i].any << std::endl;
  }
}
