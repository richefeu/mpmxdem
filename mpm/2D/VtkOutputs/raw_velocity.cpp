#include "raw_velocity.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, raw_velocity> registrar("raw_velocity");

void raw_velocity::save(std::ostream& os) {
  os << std::endl << "VECTORS raw_velocity float" << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << box->MP[i].vel << " 0" << std::endl;
  }
}